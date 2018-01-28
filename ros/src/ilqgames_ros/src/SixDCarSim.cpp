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
// Three player VICON room demo. Player ordering is given by the following:
// (P1, P2, P3) = (Car, Pedestrian 1, Pedestrian 2).
//
///////////////////////////////////////////////////////////////////////////////

#include <ilqgames/dynamics/concatenated_dynamical_system.h>
#include <ilqgames/dynamics/single_player_car_6d.h>
#include <ilqgames/solver/problem.h>
#include <ilqgames_ros/SixDCarSim.h>

#include <ilqgames_msgs/State.h>
#include <ilqgames_msgs/ThreePlayerRacingInput.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <glog/logging.h>
#include <memory>
#include <sstream>
#include <vector>

namespace ilqgames {
namespace ilqgames_ros {

bool SixDCarSim::Initialize(const ros::NodeHandle& n) {
  // setup name of node for expressive error messages
  name_ = ros::names::append(n.getNamespace(), "6DCarSim");
  // check that parameters and pubs/subs load before continuing
  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }
  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // set last time to something reasonable
  t_last_ = ros::Time::now().toSec();


  // declare pointer to problem dynamics
  const ConcatenatedDynamicalSystem& dynamics =
      *static_cast<const ConcatenatedDynamicalSystem*>(
          &problem_->Solver().Dynamics());

  // load initial state into x_
  for (PlayerIndex ii = 0; ii < dynamics.NumPlayers(); ii++) {
    x_ = problem_->InitialState();
  }


  // send initial state message
  size_t cumulative_xdim = 0;
  for (size_t ii = 0; ii < state_topics_.size(); ii++) {
    ilqgames_msgs::State msg;

    for (size_t jj = 0; jj < dynamics.SubsystemXDim(ii); jj++) {
      msg.x.push_back(x_(jj + cumulative_xdim));
    }
    state_publishers_[ii].publish(msg);
    cumulative_xdim += dynamics.SubsystemXDim(ii);
  }

  // node seems to crash here
  // declare input vectors as NaNs

  for (PlayerIndex ii = 0; ii < dynamics.NumPlayers(); ii++) {
    u_.push_back(VectorXf::Zero(dynamics.UDim(ii)));
  }
  // u_[ii] = VectorXf::Constant(dynamics.UDim(ii),
  // std::numeric_limits<float>::quiet_NaN());

  // signal completion of intialization
  initialized_ = true;
  timer_.start();
  return true;
}

bool SixDCarSim::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // read relevant rostopic names from from ros param server
  if (!nl.getParam("/planner/frames/fixed", fixed_frame_)) return false;
  if (!nl.getParam("/planner/topic/state/names", state_topics_)) return false;
  if (!nl.getParam("/planner/topic/state/tfs", player_frames_)) return false;
  if (!nl.getParam("/planner/topic/state/inputs", input_topic_)) return false;
  CHECK_EQ(state_topics_.size(), problem_->Solver().Dynamics().NumPlayers());
  if (!nl.getParam("/planner/sim_interval", sim_interval_)) return false;

  return true;
}

bool SixDCarSim::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // publishers, changed to address node crashing
  state_publishers_.emplace_back(
      nl.advertise<ilqgames_msgs::State>("/ilqgame_planner/P1", 1, false));

  state_publishers_.emplace_back(
      nl.advertise<ilqgames_msgs::State>("/ilqgame_planner/P2", 1, false));

  state_publishers_.emplace_back(
      nl.advertise<ilqgames_msgs::State>("/ilqgame_planner/P3", 1, false));
  
  
  // subscriber
  Input_sub_ =
      nl.subscribe(input_topic_.c_str(), 1, &SixDCarSim::InputCallback, this);

  // setup timer to iterate every replanning_interval_
  timer_ = nl.createTimer(ros::Duration(sim_interval_),
                          &SixDCarSim::TimerCallback, this, false, false);
  return true;
}

void SixDCarSim::TimerCallback(const ros::TimerEvent& e) {
  // might need to structure this as an if-else!
  if (!initialized_) {
    ROS_WARN_THROTTLE(1.0, "%s: Not initialized. Ignoring timer callback.",
                      name_.c_str());
    return;
  }

  const double t_ = ros::Time::now().toSec();

  // pointer to current system dynamics
  const ConcatenatedDynamicalSystem& dynamics =
      *static_cast<const ConcatenatedDynamicalSystem*>(
          &problem_->Solver().Dynamics());

  // update state using simple xdot*dt calc.
  x_ += dynamics.Evaluate(t_, x_, u_) * (t_ - t_last_);


  const std::vector<float> xPos = problem_->Xs(x_);
  const std::vector<float> yPos = problem_->Ys(x_);


  for (size_t ii = 0; ii < state_topics_.size(); ii++) {
    geometry_msgs::TransformStamped tf;
    tf.header.frame_id = fixed_frame_;
    tf.header.stamp = ros::Time::now();
    tf.child_frame_id = player_frames_[ii];

    tf.transform.translation.x = xPos[ii];
    tf.transform.translation.y = yPos[ii];
    tf.transform.translation.z = 0;
    tf.transform.rotation.x = 0;
    tf.transform.rotation.y = 0;
    tf.transform.rotation.z = 0;
    tf.transform.rotation.w = 1;
    
    tf_broadcaster_.sendTransform(tf);
  }
  // keep track of proper indeces
  size_t cumulative_xdim = 0;
  // load state msgs
  for (size_t ii = 0; ii < state_topics_.size(); ii++) {
    ilqgames_msgs::State msg;

    for (size_t jj = 0; jj < dynamics.SubsystemXDim(ii); jj++) {
      msg.x.push_back(x_(jj + cumulative_xdim));
    }
    state_publishers_[ii].publish(msg);
    cumulative_xdim += dynamics.SubsystemXDim(ii);
  }
  // update time
  t_last_ = t_;
}

void SixDCarSim::InputCallback(
    const ilqgames_msgs::ThreePlayerRacingInput::ConstPtr& msg) {
  for (size_t jj = 0; jj < u_[0].size(); jj++) {
    u_[0](jj) = msg->P1[jj];
  }

  for (size_t jj = 0; jj < u_[1].size(); jj++) {
    u_[1](jj) = msg->P2[jj];
  }

  for (size_t jj = 0; jj < u_[2].size(); jj++) {
    u_[2](jj) = msg->P3[jj];
  }
}

}  // namespace ilqgames_ros
}  // namespace ilqgames
