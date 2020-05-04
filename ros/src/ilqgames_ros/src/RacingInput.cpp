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
#include <ilqgames_ros/RacingInput.h>

#include <ilqgames_msgs/State.h>
#include <ilqgames_msgs/ThreePlayerRacingControl.h>
#include <ilqgames_msgs/ThreePlayerRacingInput.h>
#include <ros/ros.h>

#include <glog/logging.h>
#include <Eigen/Core>
#include <memory>
#include <sstream>
#include <vector>

namespace ilqgames {
namespace ilqgames_ros {

bool RacingInput::Initialize(const ros::NodeHandle& n) {
  // setup name of node for expressive error messages
  name_ = ros::names::append(n.getNamespace(), "RacingInput");
  // check that parameters and pubs/subs load before continuing

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }
  std::cout << "loaded, registered, controller" << std::endl;

  // declare local dummy of system dynamics
  const std::shared_ptr<const ConcatenatedDynamicalSystem> dynamics(
      new ConcatenatedDynamicalSystem(
          {std::make_shared<SinglePlayerCar6D>(kInterAxleLength),
           std::make_shared<SinglePlayerCar6D>(kInterAxleLength),
           std::make_shared<SinglePlayerCar6D>(kInterAxleLength)},
          kTimeStep));
  std::cout << "test1 controller" << std::endl;
  // set last time to something reasonable
  t_last_ = ros::Time::now().toSec();

  std::cout << "test2 controller" << std::endl;

  // node seem to crash here
  // declare input/Control vectors as NaNs
  for (PlayerIndex ii = 0; ii < dynamics->NumPlayers(); ii++) {
    u_.push_back(VectorXf::Constant(dynamics->UDim(ii),
                                    std::numeric_limits<float>::quiet_NaN()));

    u_ref_.push_back(VectorXf::Constant(
        dynamics->UDim(ii), std::numeric_limits<float>::quiet_NaN()));

    alpha_.push_back(VectorXf::Constant(
        dynamics->XDim(), std::numeric_limits<float>::quiet_NaN()));

    P_.push_back(VectorXf::Constant(dynamics->XDim() * dynamics->TotalUDim(),
                                    std::numeric_limits<float>::quiet_NaN()));

    x_received_.push_back(VectorXf::Constant(
        dynamics->SubsystemXDim(ii), std::numeric_limits<float>::quiet_NaN()));
  }
  std::cout << "test3 controller" << std::endl;

  x_ = VectorXf::Constant(dynamics->XDim(),
                          std::numeric_limits<float>::quiet_NaN());

  x_ref_ = VectorXf::Constant(dynamics->XDim(),
                              std::numeric_limits<float>::quiet_NaN());

  std::cout << "test4 controller" << std::endl;

  // signal completion of intialization
  initialized_ = true;
  timer_.start();

  std::cout << "u: done with initialization" << std::endl << std::flush;
  return true;
}

bool RacingInput::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  // read relevant rostopic names from from ros param server
  if (!nl.getParam("/planner/topic/state/names", state_topics_)) return false;
  if (!nl.getParam("/planner/topic/state/inputs", Input_topic_)) return false;
  if (!nl.getParam("/planner/topic/state/control", Control_topic_))
    return false;
  CHECK_EQ(state_topics_.size(), problem_->Solver().Dynamics().NumPlayers());
  // Timer interval.
  if (!nl.getParam("/planner/control_interval", control_interval_))
    return false;
  if (!nl.getParam("/planner/car/axle", kInterAxleLength)) return false;

  return true;
}

bool RacingInput::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  // publisher
  Input_pub_ = nl.advertise<ilqgames_msgs::ThreePlayerRacingInput>(
      Input_topic_.c_str(), 1, false);

  // subscribers
  Control_sub_ = nl.subscribe(Control_topic_.c_str(), 1,
                              &RacingInput::ControlCallback, this);
  // state subs, syntax changed to address node crashing error
  state_subs_.emplace_back(nl.subscribe("/ilqgame_planner/P1", 1,
                                        &RacingInput::P1StateCallback, this));

  state_subs_.emplace_back(nl.subscribe("/ilqgame_planner/P2", 1,
                                        &RacingInput::P2StateCallback, this));

  state_subs_.emplace_back(nl.subscribe("/ilqgame_planner/P3", 1,
                                        &RacingInput::P3StateCallback, this));

  /* old syntax
    state_subs_[1] = nl.subscribe(
      state_topics_[1].c_str(), 1, &RacingInput::P2StateCallback, this);
  */

  // setup timer to iterate every replanning_interval_
  timer_ = nl.createTimer(ros::Duration(control_interval_),
                          &RacingInput::TimerCallback, this, false, false);

  return true;
}

void RacingInput::TimerCallback(const ros::TimerEvent& e) {
  if (!initialized_) {
    ROS_WARN_THROTTLE(1.0, "%s: Not initialized. Ignoring timer callback.",
                      name_.c_str());
    return;
  }
  if (!ReceivedAllMsgs()) {
    ROS_WARN_THROTTLE(
        1.0, "%s: Haven't received all messages. Ignoring timer callback.",
        name_.c_str());
    return;
  }
  //
  const double t_ = ros::Time::now().toSec();

  size_t cumulative_xdim = 0;

  // construct x_ with the latest x states.
  for (size_t ii = 0; ii < u_ref_.size(); ii++) {
    for (size_t jj = 0; jj < x_received_[ii].size(); jj++) {
      x_(jj + cumulative_xdim) = x_received_[ii](jj);
    }
    cumulative_xdim += x_received_[ii].size();
  }

  delta_x_ = x_ - x_ref_;

  // calculate new control inputs
  // do these dims work out?
  u_[0] = u_ref_[0] - P_mat_[0] * delta_x_ - alpha_[0];
  u_[1] = u_ref_[1] - P_mat_[1] * delta_x_ - alpha_[1];
  u_[2] = u_ref_[2] - P_mat_[2] * delta_x_ - alpha_[2];


  // keep track of proper indeces
  // load inputs to respective elements of message
  ilqgames_msgs::ThreePlayerRacingInput Input_msg;

  for (size_t jj = 0; jj < u_[0].size(); jj++) {
    // load state msgs
    Input_msg.P1.push_back(u_[0](jj));
  }
  for (size_t jj = 0; jj < u_[1].size(); jj++) {
    // load state msgs
    Input_msg.P2.push_back(u_[1](jj));
  }
  for (size_t jj = 0; jj < u_[2].size(); jj++) {
    // load state msgs
    Input_msg.P3.push_back(u_[2](jj));
  }

  // publish!
  Input_pub_.publish(Input_msg);

  // update time
  t_last_ = t_;
}

void RacingInput::ControlCallback(
    const ilqgames_msgs::ThreePlayerRacingControl::ConstPtr& msg) {
  std::cout << "u: entering control callback" << std::endl;

  // finish input callback
  for (size_t jj = 0; jj < x_.size(); jj++) {
    x_ref_(jj) = msg->x_ref[jj];
  }

  // load contents of alpha message
  for (size_t jj = 0; jj < alpha_[0].size(); jj++) {
    alpha_[0](jj) = msg->alphaP1[jj];
  }
  for (size_t jj = 0; jj < alpha_[1].size(); jj++) {
    alpha_[1](jj) = msg->alphaP2[jj];
  }
  for (size_t jj = 0; jj < alpha_[2].size(); jj++) {
    alpha_[2](jj) = msg->alphaP3[jj];
  }

  // load u_ref values into local vars
  for (size_t jj = 0; jj < u_ref_[0].size(); jj++) {
    u_ref_[0](jj) = msg->u_refP1[jj];
  }
  for (size_t jj = 0; jj < u_ref_[1].size(); jj++) {
    u_ref_[1](jj) = msg->u_refP2[jj];
  }
  for (size_t jj = 0; jj < u_ref_[2].size(); jj++) {
    u_ref_[2](jj) = msg->u_refP3[jj];
  }

  for (size_t jj = 0; jj < P_[0].size(); jj++) {
    P_[0](jj, 0) = msg->PP1[jj];
  }
  for (size_t jj = 0; jj < P_[0].size(); jj++) {
    P_[0](jj, 0) = msg->PP1[jj];
  }
  for (size_t jj = 0; jj < P_[0].size(); jj++) {
    P_[0](jj, 0) = msg->PP1[jj];
  }

  // reshape P_ matrices to be of proper
  // might be some type conversion issues here
  const Eigen::Map<MatrixXf> P1(P_[0].data(), x_.size(), u_ref_[0].size());
  const Eigen::Map<MatrixXf> P2(P_[1].data(), x_.size(), u_ref_[1].size());
  const Eigen::Map<MatrixXf> P3(P_[2].data(), x_.size(), u_ref_[2].size());

  P_mat_[0] = P1;
  P_mat_[1] = P2;
  P_mat_[2] = P3;
}

void RacingInput::P1StateCallback(const ilqgames_msgs::State::ConstPtr& msg) {
  // finish input callback
  for (size_t jj = 0; jj < x_received_[0].size(); jj++) {
    x_received_[0](jj) = msg->x[jj];
  }
}

void RacingInput::P2StateCallback(const ilqgames_msgs::State::ConstPtr& msg) {
  // finish input callback
  for (size_t jj = 0; jj < x_received_[1].size(); jj++) {
    x_received_[1](jj) = msg->x[jj];
  }
}

void RacingInput::P3StateCallback(const ilqgames_msgs::State::ConstPtr& msg) {
  // finish input callback
  for (size_t jj = 0; jj < x_received_[2].size(); jj++) {
    x_received_[2](jj) = msg->x[jj];
  }
}

bool RacingInput::ReceivedAllMsgs() const {
  if (x_.hasNaN()) return false;
  if (x_ref_.hasNaN()) return false;

  for (size_t jj = 0; jj < u_.size(); jj++) {
    if (u_[jj].hasNaN()) return false;
    if (x_received_[jj].hasNaN()) return false;
    if (u_ref_[jj].hasNaN()) return false;
    if (alpha_[jj].hasNaN()) return false;
    if (P_mat_[jj].hasNaN()) return false;
  }

  return true;
}
}  // namespace ilqgames_ros
}  // namespace ilqgames
