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

#include <ilqgames/cost/curvature_cost.h>
#include <ilqgames/cost/final_time_cost.h>
#include <ilqgames/cost/nominal_path_length_cost.h>
#include <ilqgames/cost/proximity_cost.h>
#include <ilqgames/cost/quadratic_cost.h>
#include <ilqgames/cost/quadratic_polyline2_cost.h>
#include <ilqgames/cost/semiquadratic_cost.h>
#include <ilqgames/cost/semiquadratic_polyline2_cost.h>
#include <ilqgames/dynamics/concatenated_dynamical_system.h>
#include <ilqgames/dynamics/single_player_dubins_car.h>
#include <ilqgames/dynamics/single_player_unicycle_4d.h>
#include <ilqgames/geometry/polyline2.h>
#include <ilqgames/solver/ilq_solver.h>
#include <ilqgames/solver/problem.h>
#include <ilqgames/utils/solver_log.h>
#include <ilqgames/utils/strategy.h>
#include <ilqgames/utils/types.h>
#include <ilqgames_ros/two_player_vicon_demo.h>

#include <glog/logging.h>
#include <math.h>
#include <ros/ros.h>
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

static const Dimension kP1XIdx = P1::kPxIdx;
static const Dimension kP1YIdx = P1::kPyIdx;
static const Dimension kP1HeadingIdx = P1::kThetaIdx;
static const Dimension kP1VIdx = P1::kVIdx;

static const Dimension kP2XIdx = P1::kNumXDims + P2::kPxIdx;
static const Dimension kP2YIdx = P1::kNumXDims + P2::kPyIdx;
static const Dimension kP2HeadingIdx = P1::kNumXDims + P2::kThetaIdx;

static const Dimension kP1OmegaIdx = P1::kOmegaIdx;
static const Dimension kP1AIdx = P1::kAIdx;
static const Dimension kP2OmegaIdx = P2::kOmegaIdx;
}  // anonymous namespace

TwoPlayerViconDemo::TwoPlayerViconDemo(const ros::NodeHandle& n)
    : x_idxs_({kP1XIdx, kP2XIdx}),
      y_idxs_({kP1YIdx, kP2YIdx}),
      heading_idxs_({kP1HeadingIdx, kP2HeadingIdx}) {
  LoadParameters(n);

  // Create dynamics.
  const std::shared_ptr<const ConcatenatedDynamicalSystem> dynamics(
      new ConcatenatedDynamicalSystem(
          {std::make_shared<P1>(), std::make_shared<P2>(kDubinsV)}, kTimeStep));

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
  const Polyline2 lane1(
      {Point2(-748.684, 2322.067), Point2(0., 0.0), Point2(512.94, -1590.75)});

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
      kOmegaCostWeight, kP1OmegaIdx, 0.0, "Steering");
  const auto p1_a_cost = std::make_shared<QuadraticCost>(kACostWeight, kP1AIdx,
                                                         0.0, "Acceleration");
  p1_cost.AddControlCost(0, p1_omega_cost);
  p1_cost.AddControlCost(0, p1_a_cost);

  const auto p2_omega_cost = std::make_shared<QuadraticCost>(
      kOmegaCostWeight, kP2OmegaIdx, 0.0, "Steering");
  p2_cost.AddControlCost(1, p2_omega_cost);

  // Goal costs.
  constexpr float kFinalTimeWindow = 0.5;  // s
  const auto p1_goalx_cost = std::make_shared<FinalTimeCost>(
      std::make_shared<QuadraticCost>(kGoalCostWeight, kP1XIdx, kP1GoalX),
      kTimeHorizon - kFinalTimeWindow, "GoalX");
  const auto p1_goaly_cost = std::make_shared<FinalTimeCost>(
      std::make_shared<QuadraticCost>(kGoalCostWeight, kP1YIdx, kP1GoalY),
      kTimeHorizon - kFinalTimeWindow, "GoalY");
  p1_cost.AddStateCost(p1_goalx_cost);
  p1_cost.AddStateCost(p1_goaly_cost);

  const auto p2_goalx_cost = std::make_shared<FinalTimeCost>(
      std::make_shared<QuadraticCost>(kGoalCostWeight, kP2XIdx, kP2GoalX),
      kTimeHorizon - kFinalTimeWindow, "GoalX");
  const auto p2_goaly_cost = std::make_shared<FinalTimeCost>(
      std::make_shared<QuadraticCost>(kGoalCostWeight, kP2YIdx, kP2GoalY),
      kTimeHorizon - kFinalTimeWindow, "GoalY");
  p2_cost.AddStateCost(p2_goalx_cost);
  p2_cost.AddStateCost(p2_goaly_cost);

  // Pairwise proximity costs.
  const std::shared_ptr<ProximityCost> p1p2_proximity_cost(
      new ProximityCost(kP1ProximityCostWeight, {kP1XIdx, kP1YIdx},
                        {kP2XIdx, kP2YIdx}, kMinProximity, "ProximityP2"));
  p1_cost.AddStateCost(p1p2_proximity_cost);

  const std::shared_ptr<ProximityCost> p2p1_proximity_cost(
      new ProximityCost(kP2ProximityCostWeight, {kP2XIdx, kP2YIdx},
                        {kP1XIdx, kP1YIdx}, kMinProximity, "ProximityP1"));
  p2_cost.AddStateCost(p2p1_proximity_cost);

  // Set up solver.
  solver_.reset(
      new ILQSolver(dynamics, {p1_cost, p2_cost}, kTimeHorizon));
}

void TwoPlayerViconDemo::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  CHECK(nl.getParam("goals/p1/x", kP1GoalX));
  CHECK(nl.getParam("goals/p1/y", kP1GoalY));
  CHECK(nl.getParam("goals/p2/x", kP2GoalX));
  CHECK(nl.getParam("goals/p2/y", kP2GoalY));

  CHECK(nl.getParam("speed/min", kMinV));
  CHECK(nl.getParam("speed/max", kMaxV));
  CHECK(nl.getParam("speed/nominal", kNominalV));
  CHECK(nl.getParam("speed/dubins", kDubinsV));

  CHECK(nl.getParam("weight/u/accel", kACostWeight));
  CHECK(nl.getParam("weight/u/omega", kOmegaCostWeight));
  CHECK(nl.getParam("weight/x/v/max", kMaxVCostWeight));
  CHECK(nl.getParam("weight/x/v/nominal", kNominalVCostWeight));
  CHECK(nl.getParam("weight/x/goal", kGoalCostWeight));
  CHECK(nl.getParam("weight/x/lane/center", kLaneCostWeight));
  CHECK(nl.getParam("weight/x/lane/boundary", kLaneBoundaryCostWeight));
  CHECK(nl.getParam("weight/x/proximity/p1", kP1ProximityCostWeight));
  CHECK(nl.getParam("weight/x/proximity/p2", kP2ProximityCostWeight));

  CHECK(nl.getParam("proximity/min", kMinProximity));
  CHECK(nl.getParam("lane/half_width", kLaneHalfWidth));
}

}  // namespace ilqgames_ros
