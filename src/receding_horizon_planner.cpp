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

#include <ilqgames/solver/problem.h>
#include <ilqgames_ros/receding_horizon_planner.h>

#include <darpa_msgs/EgoState.h>
#include <darpa_msgs/EgoTrajectory.h>
#include <darpa_msgs/OtherState.h>

#include <glog/logging.h>
#include <ros/ros.h>
#include <memory>
#include <vector>

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
  for (size_t ii = 0; ii < state_types_.size(); ii++) {
    if (state_types_[ii] == "EgoState") {
      current_states_.push_back(
          VectorXf::Constant(4, std::numeric_limits<float>::quiet_NaN()));
    } else {
      CHECK_EQ(state_types_[ii], "OtherState");
      current_states_.push_back(
          VectorXf::Constant(3, std::numeric_limits<float>::quiet_NaN()));
    }
  }

  initialized_ = true;
  return true;
}

bool RecedingHorizonPlanner::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Topics.
  if (!nl.getParam("topic/traj", traj_topic_)) return false;
  if (!nl.getParam("topic/state/names", state_topics_)) return false;
  CHECK_EQ(state_topics_.size(), problem_->Solver().Dynamics().NumPlayers());

  // State types (these will get parsed later).
  if (!nl.getParam("topic/state/types", state_types_)) return false;
  CHECK_EQ(state_types_.size(), state_topics_.size());

  // Timer interval.
  if (!nl.getParam("replanning_interval", replanning_interval_)) return false;

  return true;
}

bool RecedingHorizonPlanner::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Timer.
  timer_ = nl.createTimer(ros::Duration(replanning_interval_),
                          &RecedingHorizonPlanner::TimerCallback, this);

  // Publishers.
  traj_pub_ =
      nl.advertise<darpa_msgs::EgoTrajectory>(traj_topic_.c_str(), 1, false);

  // Subscribers.
  for (size_t ii = 0; ii < state_topics_.size(); ii++) {
    const auto& topic = state_topics_[ii];
    const auto& type = state_types_[ii];

    // Generate a lambda function for this callback, parsing the state type.
    // NOTE: this could be implemented with templated lambdas but that's a
    // relatively new feature, so we'll avoid using it in case we need to use an
    // older compiler.
    if (type == "EgoState") {
      boost::function<void(const darpa_msgs::EgoState::ConstPtr&, size_t)>
          callback =
              [this](const darpa_msgs::EgoState::ConstPtr& msg, size_t idx) {
                StateCallback(msg, idx);
              };  // callback

      // Create a new subscriber with this callback.
      state_subs_.emplace_back(nl.subscribe<darpa_msgs::EgoState>(
          topic, 1, boost::bind(callback, _1, ii)));
    } else {
      CHECK_EQ(type, "OtherState");

      boost::function<void(const darpa_msgs::OtherState::ConstPtr&, size_t)>
          callback =
              [this](const darpa_msgs::OtherState::ConstPtr& msg, size_t idx) {
                StateCallback(msg, idx);
              };  // callback

      state_subs_.emplace_back(nl.subscribe<darpa_msgs::OtherState>(
          topic, 1, boost::bind(callback, _1, ii)));
    }
  }

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

  // Parse state information into big vector.
  VectorXf x0(problem_->Solver().Dynamics().XDim());
  size_t dims_so_far = 0;
  for (const auto& x : current_states_) {
    x0.segment(dims_so_far, x.size()) = x;
    dims_so_far += x.size();
  }

  // Set up next receding horizon problem and solve.
  problem_->SetUpNextRecedingHorizon(x0, ros::Time::now().toSec(),
                                     replanning_interval_);
  const auto log = problem_->Solve();

  // Splice in new solution. Handle first time through separately.
  if (!solution_splicer_.get())
    solution_splicer_.reset(new SolutionSplicer(*log));
  else
    solution_splicer_->Splice(*log, ros::Time::now().toSec());

  // Overwrite problem with spliced solution.
  problem_->OverwriteSolution(solution_splicer_->CurrentOperatingPoint(),
                              solution_splicer_->CurrentStrategies());
}

void RecedingHorizonPlanner::StateCallback(
    const darpa_msgs::EgoState::ConstPtr& msg, size_t idx) {
  auto& x = current_states_[idx];
  CHECK_EQ(x.size(), 4);

  x(0) = msg->x;
  x(1) = msg->y;
  x(2) = msg->theta;
  x(3) = msg->v;
}

void RecedingHorizonPlanner::StateCallback(
    const darpa_msgs::OtherState::ConstPtr& msg, size_t idx) {
  auto& x = current_states_[idx];
  CHECK_EQ(x.size(), 3);

  x(0) = msg->x;
  x(1) = msg->y;
  x(2) = msg->theta;
}

bool RecedingHorizonPlanner::ReceivedAllStateUpdates() const {
  for (const auto& x : current_states_) {
    if (x.hasNaN()) return false;
  }

  return true;
}

}  // namespace ilqgames_ros
