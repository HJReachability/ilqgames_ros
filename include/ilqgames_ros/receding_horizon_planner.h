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

#ifndef ILQGAMES_ROS_RECEDING_HORIZON_PLANNER_H
#define ILQGAMES_ROS_RECEDING_HORIZON_PLANNER_H

#include <ilqgames/solver/problem.h>
#include <ilqgames/solver/solution_splicer.h>
#include <ilqgames_ros/two_player_boeing_demo.h>

#include <darpa_msgs/EgoState.h>
#include <darpa_msgs/OtherState.h>

#include <glog/logging.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <memory>
#include <vector>

namespace ilqgames_ros {

using Eigen::VectorXf;
using ilqgames::Problem;
using ilqgames::SolutionSplicer;

class RecedingHorizonPlanner {
 public:
  ~RecedingHorizonPlanner() {}
  RecedingHorizonPlanner(const std::shared_ptr<Problem>& problem)
      : problem_(problem), initialized_(false), is_first_timer_callback_(true) {
    CHECK_NOTNULL(problem.get());
  }

  // Initialize this class with all parameters and callbacks.
  bool Initialize(const ros::NodeHandle& n);

 private:
  // Load parameters and register callbacks.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Replan on a timer.
  void TimerCallback(const ros::TimerEvent& e);

  // Callback for processing the given player's state msg.
  // One callback for each type of state msg we might have.
  void StateCallback(const darpa_msgs::EgoState::ConstPtr& msg, size_t idx);
  void StateCallback(const darpa_msgs::OtherState::ConstPtr& msg, size_t idx);

  // Check to see if we've received state updates for all vehicles.
  bool ReceivedAllStateUpdates() const;

  // Plans trajectory.
  void Plan();

  // Planning problem.
  std::shared_ptr<TwoPlayerBoeingDemo> problem_;

  // Solution splicer to keep track of solution over multiple receding horizon
  // solver invocations.
  std::unique_ptr<SolutionSplicer> solution_splicer_;

  // Fixed frame id.
  std::string fixed_frame_;

  // Set a recurring timer for a discrete-time controller.
  ros::Timer timer_;
  float replanning_interval_;

  // Publishers/subscribers and related topics.
  ros::Publisher traj_pub_;
  ros::Publisher traj_viz_pub_;
  std::vector<ros::Subscriber> state_subs_;

  std::string traj_topic_;
  std::string traj_viz_topic_;
  std::vector<std::string> state_topics_;
  std::vector<std::string> state_types_;

  // List of most recent states for each system.
  std::vector<VectorXf> current_states_;

  // Naming and initialization.
  std::string name_;
  bool initialized_;
  bool is_first_timer_callback_;
};

}  // namespace ilqgames_ros

#endif
