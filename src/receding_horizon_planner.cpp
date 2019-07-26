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
#include <darpa_msgs/OtherState.h>

#include <glog/logging.h>
#include <ros/ros.h>
#include <memory>
#include <vector>

namespace ilqgames_ros {

bool Initialize(const ros::NodeHandle& n) {
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

  initialized_ = true;
  return true;
}

bool LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Topics.
  if (!nl.getParam("topic/traj", traj_topic_)) return false;
  if (!nl.getParam("topic/state/names", state_topics_)) return false;
  CHECK_EQ(state_topics_.size(), problem_->Solver().Dynamics().NumPlayers());

  // State types (these will get parsed later).
  if (!nl.getParam("topic/state/types", state_types_)) return false;

  // Timer interval.
  if (!nl.getParam("replanning_interval", replanning_interval_)) return false;

  return true;
}

bool RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Publishers.
  traj_pub_ =
      nl.advertise<fastrack_msgs::Trajectory>(traj_topic_.c_str(), 1, false);

  // Subscribers.
  replan_request_sub_ = nl.subscribe(replan_request_topic_.c_str(), 1,
                                     &Replanner::ReplanRequestCallback, this);

  // Services.
  ros::service::waitForService(replan_srv_name_);
  replan_srv_ =
      nl.serviceClient<fastrack_srvs::Replan>(replan_srv_name_.c_str(), true);

  return true;
}

void TimerCallback(const ros::TimerEvent& e);

void StateCallback(const darpa_msgs::EgoState::ConstPtr& msg);
void StateCallback(const darpa_msgs::OtherState::ConstPtr& msg);

}  // namespace ilqgames_ros
