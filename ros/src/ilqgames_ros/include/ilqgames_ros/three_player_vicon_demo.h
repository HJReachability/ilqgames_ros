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
// Three player VICON room demo.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_ROS_THREE_PLAYER_VICON_DEMO_H
#define ILQGAMES_ROS_THREE_PLAYER_VICON_DEMO_H

#include <ilqgames/solver/top_down_renderable_problem.h>
#include <ilqgames/utils/types.h>

#include <ros/ros.h>

namespace ilqgames_ros {

using namespace ilqgames;

class ThreePlayerViconDemo : public TopDownRenderableProblem {
 public:
  ~ThreePlayerViconDemo() {}
  ThreePlayerViconDemo(const ros::NodeHandle& n);

  // Accessors.
  std::vector<float> Xs(const VectorXf& x) const {
    std::vector<float> xs;
    for (auto xidx : x_idxs_) xs.push_back(x(xidx));
    return xs;
  }
  std::vector<float> Ys(const VectorXf& x) const {
    std::vector<float> ys;
    for (auto yidx : y_idxs_) ys.push_back(x(yidx));
    return ys;
  }
  std::vector<float> Thetas(const VectorXf& x) const {
    std::vector<float> thetas;
    for (auto idx : heading_idxs_) thetas.push_back(x(idx));
    return thetas;
  }

 private:
  // Load parameters from the ROS parameter server.
  void LoadParameters(const ros::NodeHandle& n);

  // Goal locations.
  float kP1GoalX, kP1GoalY, kP2GoalX, kP2GoalY, kP3GoalX, kP3GoalY;

  // Min/max/nominal speeds.
  float kMaxV, kMinV, kNominalV, kDubinsV;

  // Control cost weights.
  float kACostWeight, kOmegaCostWeight;

  // Speed and goal weights.
  float kMaxVCostWeight, kNominalVCostWeight, kGoalCostWeight;

  // Lane weights.
  float kLaneHalfWidth, kLaneCostWeight, kLaneBoundaryCostWeight;

  // Proximity weights.
  float kMinProximity, kP1ProximityCostWeight, kP2ProximityCostWeight,
      kP3ProximityCostWeight;

  // Indices for x/y/heading.
  const std::vector<Dimension> x_idxs_;
  const std::vector<Dimension> y_idxs_;
  const std::vector<Dimension> heading_idxs_;
};  // class ThreePlayerViconDemo

}  // namespace ilqgames_ros

#endif
