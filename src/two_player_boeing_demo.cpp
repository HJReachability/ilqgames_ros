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
// Three player BOEING room demo. Player ordering is given by the following:
// (P1, P2, P3) = (Car, Pedestrian 1, Pedestrian 2).
//
///////////////////////////////////////////////////////////////////////////////

#include <darpa_msgs/Polyline.h>
#include <glog/logging.h>
#include <ilqgames/cost/curvature_cost.h>
#include <ilqgames/cost/final_time_cost.h>
#include <ilqgames/cost/nominal_path_length_cost.h>
#include <ilqgames/cost/proximity_cost.h>
#include <ilqgames/cost/quadratic_cost.h>
#include <ilqgames/cost/quadratic_difference_cost.h>
#include <ilqgames/cost/quadratic_polyline2_cost.h>
#include <ilqgames/cost/semiquadratic_cost.h>
#include <ilqgames/cost/semiquadratic_polyline2_cost.h>
#include <ilqgames/dynamics/concatenated_dynamical_system.h>
#include <ilqgames/dynamics/single_player_dubins_car.h>
#include <ilqgames/dynamics/single_player_unicycle_4d.h>
#include <ilqgames/geometry/polyline2.h>
#include <ilqgames/solver/ilq_solver.h>
#include <ilqgames/solver/linesearching_ilq_solver.h>
#include <ilqgames/solver/problem.h>
#include <ilqgames/utils/solver_log.h>
#include <ilqgames/utils/strategy.h>
#include <ilqgames/utils/types.h>
#include <ilqgames_ros/two_player_boeing_demo.h>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <memory>
#include <vector>

namespace ilqgames_ros {

namespace {
// Time.
static constexpr Time kTimeStep = 0.1;      // s
static constexpr Time kTimeHorizon = 10.0;  // s
static constexpr size_t kNumTimeSteps =
    static_cast<size_t>(kTimeHorizon / kTimeStep);

// Flag for one-sided costs.
static constexpr bool kOrientedRight = true;

// State dimensions.
using P1 = SinglePlayerUnicycle4D;
using P2 = SinglePlayerDubinsCar;

static constexpr size_t kP1XIdx = P1::kPxIdx;
static constexpr size_t kP1YIdx = P1::kPyIdx;
static constexpr size_t kP1HeadingIdx = P1::kThetaIdx;
static constexpr size_t kP1VIdx = P1::kVIdx;

static constexpr size_t kP2XIdx = P1::kNumXDims + P2::kPxIdx;
static constexpr size_t kP2YIdx = P1::kNumXDims + P2::kPyIdx;
static constexpr size_t kP2HeadingIdx = P1::kNumXDims + P2::kThetaIdx;

static constexpr size_t kP1OmegaIdx = P1::kOmegaIdx;
static constexpr size_t kP1AIdx = P1::kAIdx;
static constexpr size_t kP2OmegaIdx = P2::kOmegaIdx;
}  // anonymous namespace

TwoPlayerBoeingDemo::TwoPlayerBoeingDemo(const ros::NodeHandle& n)
    : x_idxs_({kP1XIdx, kP2XIdx}),
      y_idxs_({kP1YIdx, kP2YIdx}),
      heading_idxs_({kP1HeadingIdx, kP2HeadingIdx}) {
  LoadParameters(n);

  // Create dynamics.
  const std::shared_ptr<ConcatenatedDynamicalSystem> dynamics(
      new ConcatenatedDynamicalSystem({
          std::make_shared<P1>(),
          std::make_shared<P2>(kDubinsV),
      }));

  // Set up initial state.
  // NOTE: this will get overwritten before the solver is actually called.
  x0_ = VectorXf::Constant(dynamics->XDim(),
                           std::numeric_limits<float>::quiet_NaN());

  // Set up initial strategies and operating point.
  strategies_.reset(new std::vector<Strategy>());
  for (size_t ii = 0; ii < dynamics->NumPlayers(); ii++)
    strategies_->emplace_back(kNumTimeSteps, dynamics->XDim(),
                              dynamics->UDim(ii));

  operating_point_.reset(
      new OperatingPoint(kNumTimeSteps, dynamics->NumPlayers(), 0.0, dynamics));

  // Set up costs for all players.
  PlayerCost p1_cost, p2_cost;

  // Stay in lanes.
  const Polyline2 lane1(lane_positions_);

  const std::shared_ptr<QuadraticPolyline2Cost> p1_lane_cost(
      new QuadraticPolyline2Cost(kLaneCostWeight, lane1, {kP1XIdx, kP1YIdx},
                                 "LaneCenter"));
  const std::shared_ptr<SemiquadraticPolyline2Cost> p1_lane_r_cost(
      new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, lane1,
                                     {kP1XIdx, kP1YIdx}, kLaneHalfWidth,
                                     kOrientedRight, "LaneRightBoundary"));
  const std::shared_ptr<SemiquadraticPolyline2Cost> p1_lane_l_cost(
      new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, lane1,
                                     {kP1XIdx, kP1YIdx}, -kLaneHalfWidth,
                                     !kOrientedRight, "LaneLeftBoundary"));
  p1_cost.AddStateCost(p1_lane_cost);
  p1_cost.AddStateCost(p1_lane_r_cost);
  p1_cost.AddStateCost(p1_lane_l_cost);

  // Max/min/nominal speed costs.
  const auto p1_min_v_cost = std::make_shared<SemiquadraticCost>(
      kMaxVCostWeight, kP1VIdx, kMinV, !kOrientedRight, "MinV");
  const auto p1_max_v_cost = std::make_shared<SemiquadraticCost>(
      kMaxVCostWeight, kP1VIdx, kMaxV, kOrientedRight, "MaxV");
  const auto p1_nominal_v_cost = std::make_shared<QuadraticCost>(
      kNominalVCostWeight, kP1VIdx, kNominalV, "NominalV");
  p1_cost.AddStateCost(p1_min_v_cost);
  p1_cost.AddStateCost(p1_max_v_cost);
  p1_cost.AddStateCost(p1_nominal_v_cost);

  // Penalize control effort.
  const auto p1_omega_cost = std::make_shared<QuadraticCost>(
      kP1OmegaCostWeight, kP1OmegaIdx, 0.0, "Steering");
  const auto p1_accel_cost = std::make_shared<SemiquadraticCost>(
      kP1AccelCostWeight, kP1AIdx, 0.0, kOrientedRight, "Acceleration");
  const auto p1_decel_cost = std::make_shared<SemiquadraticCost>(
      kP1DecelCostWeight, kP1AIdx, 0.0, !kOrientedRight, "Deceleration");
  p1_cost.AddControlCost(0, p1_omega_cost);
  p1_cost.AddControlCost(0, p1_accel_cost);
  p1_cost.AddControlCost(0, p1_decel_cost);

  const auto p2_omega_cost = std::make_shared<QuadraticCost>(
      kP2OmegaCostWeight, kP2OmegaIdx, 0.0, "Steering");
  p2_cost.AddControlCost(1, p2_omega_cost);

  // Goal costs.
  p1_goalx_cost_ = std::make_shared<FinalTimeCost>(
      std::make_shared<QuadraticCost>(kP1GoalCostWeight, kP1XIdx, kP1GoalX),
      kTimeHorizon - kGoalFinalTimeWindow, "GoalX");
  p1_goaly_cost_ = std::make_shared<FinalTimeCost>(
      std::make_shared<QuadraticCost>(kP1GoalCostWeight, kP1YIdx, kP1GoalY),
      kTimeHorizon - kGoalFinalTimeWindow, "GoalY");
  p1_cost.AddStateCost(p1_goalx_cost_);
  p1_cost.AddStateCost(p1_goaly_cost_);

  // Pairwise proximity costs.
  const std::shared_ptr<ProximityCost> p1p2_proximity_cost(
      new ProximityCost(kP1ProximityCostWeight, {kP1XIdx, kP1YIdx},
                        {kP2XIdx, kP2YIdx}, kP1AvoidanceMargin, "ProximityP2"));
  p1_cost.AddStateCost(p1p2_proximity_cost);

  p2_proximity_cost_ = std::make_shared<FinalTimeCost>(
      std::shared_ptr<ProximityCost>(new ProximityCost(
          kP2ProximityCostWeight, {kP2XIdx, kP2YIdx}, {kP1XIdx, kP1YIdx},
          kP2AvoidanceMargin, "ProximityP1")),
      kTimeHorizon - kProximityFinalTimeWindow);
  p2_cost.AddStateCost(p2_proximity_cost_);

  // Adversarial cost.
  p2_adversarial_cost_ = std::make_shared<InitialTimeCost>(
      std::shared_ptr<QuadraticDifferenceCost>(new QuadraticDifferenceCost(
          kP2ProximityCostWeight, {kP2XIdx, kP2YIdx}, {kP1XIdx, kP1YIdx},
          "Adversarial")),
      kAdversarialInitialTimeWindow);
  p2_cost.AddStateCost(p2_adversarial_cost_);

  // Desired heading cost for P2.
  const auto p2_nominal_heading_cost = std::make_shared<QuadraticCost>(
      kP2NominalHeadingCostWeight, kP2HeadingIdx, kP2NominalHeading, "Heading");
  p2_cost.AddStateCost(p2_nominal_heading_cost);

  // Set up solver.
  solver_.reset(new LinesearchingILQSolver(dynamics, {p1_cost, p2_cost},
                                           kTimeHorizon, kTimeStep));
}

void TwoPlayerBoeingDemo::UpdateTimeBasedCosts(Time start_time) {
  p1_goalx_cost_->ResetThresholdTime(start_time + kTimeHorizon -
                                     kGoalFinalTimeWindow);
  p1_goaly_cost_->ResetThresholdTime(start_time + kTimeHorizon -
                                     kGoalFinalTimeWindow);
  p2_proximity_cost_->ResetThresholdTime(start_time + kTimeHorizon -
                                         kProximityFinalTimeWindow);
  p2_adversarial_cost_->ResetThresholdTime(start_time +
                                           kAdversarialInitialTimeWindow);
}

void TwoPlayerBoeingDemo::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // CHECK(nl.getParam("goals/p1/x", kP1GoalX));
  // CHECK(nl.getParam("goals/p1/y", kP1GoalY));
  // CHECK(nl.getParam("goals/p2/x", kP2GoalX));
  // CHECK(nl.getParam("goals/p2/y", kP2GoalY));

  CHECK(nl.getParam("speed/min", kMinV));
  CHECK(nl.getParam("speed/max", kMaxV));
  CHECK(nl.getParam("speed/nominal", kNominalV));
  CHECK(nl.getParam("speed/dubins", kDubinsV));

  CHECK(nl.getParam("weight/ego/u/accel", kP1AccelCostWeight));
  CHECK(nl.getParam("weight/ego/u/decel", kP1DecelCostWeight));
  CHECK(nl.getParam("weight/ego/u/omega", kP1OmegaCostWeight));
  CHECK(nl.getParam("weight/ego/x/v/max", kMaxVCostWeight));
  CHECK(nl.getParam("weight/ego/x/v/nominal", kNominalVCostWeight));
  CHECK(nl.getParam("weight/ego/x/goal", kP1GoalCostWeight));
  CHECK(nl.getParam("weight/ego/x/lane/center", kLaneCostWeight));
  CHECK(nl.getParam("weight/ego/x/lane/boundary", kLaneBoundaryCostWeight));
  CHECK(nl.getParam("weight/ego/x/proximity", kP1ProximityCostWeight));

  CHECK(nl.getParam("weight/other/u/omega", kP2OmegaCostWeight));
  CHECK(nl.getParam("weight/other/x/proximity", kP2ProximityCostWeight));
  CHECK(nl.getParam("weight/other/x/adversarial", kP2AdversarialCostWeight));
  CHECK(nl.getParam("weight/other/x/heading", kP2NominalHeadingCostWeight));

  CHECK(nl.getParam("heading/other/nominal", kP2NominalHeading));

  CHECK(nl.getParam("avoidance_margin/ego", kP1AvoidanceMargin));
  CHECK(nl.getParam("avoidance_margin/other", kP2AvoidanceMargin));
  CHECK(nl.getParam("lane/half_width", kLaneHalfWidth));

  CHECK(nl.getParam("window/goal", kGoalFinalTimeWindow));
  CHECK(nl.getParam("window/adversarial", kAdversarialInitialTimeWindow));
  kProximityFinalTimeWindow = kTimeHorizon - kAdversarialInitialTimeWindow;

  // Get lane position.
  std::string lane_srv_name;
  CHECK(nl.getParam("srv/polyline_position", lane_srv_name));
  ros::ServiceClient lane_srv =
      nl.serviceClient<darpa_msgs::Polyline>(lane_srv_name.c_str());
  lane_srv.waitForExistence();

  darpa_msgs::Polyline srv;
  while (srv.response.x_positions.empty()) {
    CHECK(lane_srv.call(srv));
    CHECK_EQ(srv.response.x_positions.size(), srv.response.y_positions.size());

    // If empty response, pause before continuing.
    if (srv.response.x_positions.empty()) {
      constexpr double kPauseTime = 1.0;
      ros::Duration(kPauseTime).sleep();
    }
  }

  for (size_t ii = 0; ii < srv.response.x_positions.size(); ii++) {
    lane_positions_.emplace_back(srv.response.x_positions[ii],
                                 srv.response.y_positions[ii]);
  }

  // Set goal to be final point along trajectory.
  kP1GoalX = lane_positions_.back().x();
  kP1GoalY = lane_positions_.back().y();
}

}  // namespace ilqgames_ros
