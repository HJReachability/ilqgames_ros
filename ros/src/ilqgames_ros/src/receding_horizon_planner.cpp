/*
 * Copyright (c) 2019, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Receding horizon planner, used for all experiments.
//
///////////////////////////////////////////////////////////////////////////////

#include <ilqgames/dynamics/single_player_car_6d.h>
#include <ilqgames/dynamics/single_player_dubins_car.h>
#include <ilqgames/dynamics/single_player_unicycle_4d.h>
#include <ilqgames/solver/problem.h>

#include <ilqgames_msgs/State.h>
#include <ilqgames_msgs/ThreePlayerRacingControl.h>
#include <ilqgames_ros/receding_horizon_planner.h>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <glog/logging.h>
#include <Eigen/Core>
#include <memory>
#include <vector>

namespace ilqgames {
namespace ilqgames_ros {

bool RecedingHorizonPlanner::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "receding_horizon_planner");

  // Load parameters.
  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  // Register callbacks.
  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Set up 'current_states_' to be a bunch of NaNs.
  const ConcatenatedDynamicalSystem& dynamics =
      *static_cast<const ConcatenatedDynamicalSystem*>(
          &problem_->Solver().Dynamics());
  for (PlayerIndex ii = 0; ii < dynamics.NumPlayers(); ii++) {
    current_states_.push_back(VectorXf::Constant(
        dynamics.SubsystemXDim(ii), std::numeric_limits<float>::quiet_NaN()));
  }

  // Initialize size of P and alpha arrays.
  P_.resize(state_topics_.size());
  alpha_.resize(state_topics_.size());

  initialized_ = true;
  return true;
}

bool RecedingHorizonPlanner::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  // Frames.
  if (!nl.getParam("/planner/frames/fixed", fixed_frame_)) return false;
  // Topics.
  if (!nl.getParam("/planner/viz/traj", traj_viz_topic_)) return false;
  if (!nl.getParam("/planner/topic/state/names", state_topics_)) return false;
  if (!nl.getParam("/planner/topic/state/control", Control_topic_))
    return false;
  CHECK_EQ(state_topics_.size(), problem_->Solver().Dynamics().NumPlayers());
  // Timer interval.
  if (!nl.getParam("/planner/replanning_interval", replanning_interval_))
    return false;
  return true;
}

bool RecedingHorizonPlanner::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Publishers.
  traj_viz_pub_ = nl.advertise<visualization_msgs::Marker>(
      traj_viz_topic_.c_str(), 1, false);

  // publish new control parameters to the input calculator
  control_pub_ = nl.advertise<ilqgames_msgs::ThreePlayerRacingControl>(
      "/new_control", 1, false);

  // Subscribers.
  for (size_t ii = 0; ii < state_topics_.size(); ii++) {
    const auto& topic = state_topics_[ii];

    // Generate a lambda function for this callback.
    boost::function<void(const ilqgames_msgs::State::ConstPtr&, size_t)>
        callback = [this](const ilqgames_msgs::State::ConstPtr& msg,
                          size_t idx) { StateCallback(msg, idx); };

    // Create a new subscriber with this callback.
    state_subs_.emplace_back(nl.subscribe<ilqgames_msgs::State>(
        topic, 1, boost::bind(callback, _1, ii)));
  }

  // Timer.
  timer_ = nl.createTimer(ros::Duration(replanning_interval_),
                          &RecedingHorizonPlanner::TimerCallback, this, false,
                          false);

  return true;
}

void RecedingHorizonPlanner::TimerCallback(const ros::TimerEvent& e) {
  if (!initialized_) {
    ROS_WARN_THROTTLE(1.0, "%s: Not initialized. Ignoring timer callback.",
                      name_.c_str());
    return;
  }

  if (!ReceivedAllStateUpdates()) {
    ROS_WARN_THROTTLE(1.0, "%s: State incomplete. Ignoring timer callback.",
                      name_.c_str());
    return;
  }

  Plan();
}

void RecedingHorizonPlanner::StateCallback(
    const ilqgames_msgs::State::ConstPtr& msg, size_t idx) {
  auto& x = current_states_[idx];
  CHECK_EQ(x.size(), msg->x.size());

  for (size_t ii = 0; ii < x.size(); ii++) x(ii) = msg->x[ii];

  // Start planning if we've never planned before and state updates have
  // been received for all players.
  if (is_first_timer_callback_ && ReceivedAllStateUpdates()) {
    Plan();
    is_first_timer_callback_ = false;
    timer_.start();
  }
}

bool RecedingHorizonPlanner::ReceivedAllStateUpdates() const {
  for (const auto& x : current_states_) {
    if (x.hasNaN()) return false;
  }

  return true;
}

void RecedingHorizonPlanner::Plan() {
  const double t = ros::Time::now().toSec();

  // Parse state information into big vector.
  VectorXf x0(problem_->Solver().Dynamics().XDim());
  size_t dims_so_far = 0;
  for (const auto& x : current_states_) {
    x0.segment(dims_so_far, x.size()) = x;
    dims_so_far += x.size();
  }

  // First timer callback, reset initial time.
  double planner_runtime = replanning_interval_;
  double warm_start_dt = replanning_interval_;
  if (is_first_timer_callback_) {
    ROS_INFO("First plan");
    problem_->ResetInitialTime(t);
    problem_->ResetInitialState(x0);
    warm_start_dt = ilqgames::constants::kSmallNumber;
    //    planner_runtime = std::numeric_limits<double>::infinity();
    is_first_timer_callback_ = false;
  } else {
    // Set up next receding horizon problem and solve.
    problem_->SetUpNextRecedingHorizon(x0, t, warm_start_dt);
    //    problem_->SetUpNextRecedingHorizon(x0, t, 0.0);
  }

  // Solve the problem.
  const ros::Time solve_start_time = ros::Time::now();
  const auto log = problem_->Solve(planner_runtime);
  ROS_INFO("%s: Planning time was %f s.", name_.c_str(),
           (ros::Time::now() - solve_start_time).toSec());

  // Splice in new solution. Handle first time through separately.
  if (!solution_splicer_.get())
    solution_splicer_.reset(new SolutionSplicer(*log));
  else
    solution_splicer_->Splice(*log, x0, problem_->Solver().Dynamics());

  // Overwrite problem with spliced solution.
  problem_->OverwriteSolution(solution_splicer_->CurrentOperatingPoint(),
                              solution_splicer_->CurrentStrategies());

  // Publish.
  Publish();
}

void RecedingHorizonPlanner::Publish() {
  const ConcatenatedDynamicalSystem& dynamics =
      *static_cast<const ConcatenatedDynamicalSystem*>(
          &problem_->Solver().Dynamics());

  //std::cout << solution_splicer_.get() << std::endl;

  if (!solution_splicer_.get()) return;
  // Exit early if we don't have any subscribers.
  // if (traj_viz_pub_.getNumSubscribers() == 0) {
  //   ROS_INFO("%s: No subscribers, so skipping visualization.",
  //   name_.c_str());
  //   return;
  // }

  // Extract current operating point.
  CHECK(solution_splicer_.get());
  const auto& traj = solution_splicer_->CurrentOperatingPoint();
  const auto& strat = solution_splicer_->CurrentStrategies();


  // Vizualize everybody's trajectory.
  std::vector<visualization_msgs::Marker> spheres(state_topics_.size());
  std::vector<visualization_msgs::Marker> lines(state_topics_.size());
  ilqgames_msgs::ThreePlayerRacingControl Control_msg;

  const auto pub_time = ros::Time::now();
  for (size_t ii = 0; ii < spheres.size(); ii++) {
    auto& s = spheres[ii];
    auto& l = lines[ii];

    auto c = std_msgs::ColorRGBA();
    c.r = static_cast<double>(ii) / (spheres.size() - 1);
    c.g = 0.1;
    c.b = 1.0 - c.r;
    c.a = 1.0;

    s.header.frame_id = fixed_frame_.c_str();
    s.header.stamp = pub_time;
    s.ns = "spheres";
    s.id = ii;
    s.type = visualization_msgs::Marker::SPHERE_LIST;
    s.action = visualization_msgs::Marker::ADD;
    s.scale.x = .5;
    s.scale.y = .5;
    s.scale.z = .5;
    s.color = c;

    l.header.frame_id = fixed_frame_.c_str();
    l.header.stamp = pub_time;
    l.ns = "lines";
    l.id = ii;
    l.type = visualization_msgs::Marker::LINE_STRIP;
    l.action = visualization_msgs::Marker::ADD;
    l.scale.x = 0.2;
    l.scale.y = 0.2;
    l.scale.z = 0.2;
    l.color = c;
  }

  for (size_t kk = 0; kk < traj.xs.size(); kk++) {
    // Extract position for each player.
    const std::vector<float> xs = problem_->Xs(traj.xs[kk]);
    const std::vector<float> ys = problem_->Ys(traj.xs[kk]);

    // ii index players, kk index time
    for (size_t ii = 0; ii < state_topics_.size(); ii++) {
      P_[ii] = strat[ii].Ps[0];
      alpha_[ii] = strat[ii].alphas[0];
      CHECK_EQ(alpha_[ii].size(), dynamics.UDim(ii));
      std::cout<<"P1 size, planner= "<<P1.size()<<std::endl;
    }

    CHECK_EQ(xs.size(), spheres.size());
    for (size_t ii = 0; ii < spheres.size(); ii++) {
      geometry_msgs::Point p;
      p.x = xs[ii];
      p.y = ys[ii];
      p.z = 0.0;

      spheres[ii].points.push_back(p);
      lines[ii].points.push_back(p);
    }
  }

  // load ref variables for control message
  x_ref = traj.xs[0];
  u_refP1 = traj.us[0][0];
  u_refP2 = traj.us[0][1];
  u_refP3 = traj.us[0][2];

  // load control message
  // load P matrices
  //should this be size udim by xdim?
  
  const Eigen::Map<MatrixXf> P1(P_[0].data(),
                                dynamics.XDim() * dynamics.TotalUDim(), 1);
  for (size_t jj = 0; jj < dynamics.XDim() * dynamics.TotalUDim(); jj++) {
    Control_msg.PP1.push_back(P1(jj, 0));
  }
  const Eigen::Map<MatrixXf> P2(P_[1].data(),
                                dynamics.XDim() * dynamics.TotalUDim(), 1);
  for (size_t jj = 0; jj < dynamics.XDim() * dynamics.TotalUDim(); jj++) {
    Control_msg.PP2.push_back(P2(jj, 0));
  }
  const Eigen::Map<MatrixXf> P3(P_[2].data(),
                                dynamics.XDim() * dynamics.TotalUDim(), 1);
  for (size_t jj = 0; jj < dynamics.XDim() * dynamics.TotalUDim(); jj++) {
    Control_msg.PP3.push_back(P3(jj, 0));
  }


  // load alphas and x_ref
  for (size_t jj = 0; jj < dynamics.XDim(); jj++) {
    Control_msg.x_ref.push_back(x_ref(jj));
  }

  // load u_ref values
  for (size_t jj = 0; jj < dynamics.UDim(0); jj++) {
    Control_msg.alphaP1.push_back(alpha_[0](jj));
    Control_msg.u_refP1.push_back(u_refP1(jj));
  }
  for (size_t jj = 0; jj < dynamics.UDim(1); jj++) {
    Control_msg.alphaP2.push_back(alpha_[1](jj));
    Control_msg.u_refP2.push_back(u_refP2(jj));
  }
  for (size_t jj = 0; jj < dynamics.UDim(2); jj++) {
    Control_msg.alphaP3.push_back(alpha_[2](jj));
    Control_msg.u_refP3.push_back(u_refP3(jj));
  }

  // Publish!
  for (size_t ii = 0; ii < spheres.size(); ii++) {
    traj_viz_pub_.publish(spheres[ii]);
    traj_viz_pub_.publish(lines[ii]);
  }

  control_pub_.publish(Control_msg);
}

}  // namespace ilqgames_ros
}  // namespace ilqgames
