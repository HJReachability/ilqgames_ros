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

#include <ilqgames/dynamics/concatenated_dynamical_system.h>
#include <ilqgames/solver/solution_splicer.h>
#include <ilqgames/solver/top_down_renderable_problem.h>
#include <ilqgames/utils/types.h>

#include <ilqgames_msgs/State.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <glog/logging.h>
#include <memory>
#include <vector>

namespace ilqgames {
namespace ilqgames_ros {

class RecedingHorizonPlanner {
 public:
  ~RecedingHorizonPlanner() {}
  RecedingHorizonPlanner(
      const std::shared_ptr<TopDownRenderableProblem>& problem)
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
  void StateCallback(const ilqgames_msgs::State::ConstPtr& msg, size_t idx);

  // Check to see if we've received state updates for all players.
  bool ReceivedAllStateUpdates() const;

  // Plan trajectory.
  void Plan();

  // Visualize the current operating point.
  void Visualize() const;

  // Planning problem.
  std::shared_ptr<TopDownRenderableProblem> problem_;

  // Solution splicer to keep track of solution over multiple receding horizon
  // solver invocations.
  std::unique_ptr<SolutionSplicer> solution_splicer_;

  // Set a recurring timer for a discrete-time controller.
  ros::Timer timer_;
  float replanning_interval_;

  // Fixed frame id.
  std::string fixed_frame_;

  // Publishers/subscribers and related topics.
  ros::Publisher traj_viz_pub_;
  std::vector<ros::Subscriber> state_subs_;

  std::string traj_viz_topic_;
  std::vector<std::string> state_topics_;

  // List of most recent states for each system.
  std::vector<VectorXf> current_states_;

  // Naming and initialization.
  std::string name_;
  bool initialized_;
  bool is_first_timer_callback_;
};

}  // namespace ilqgames_ros
}  // namespace ilqgames

#endif
