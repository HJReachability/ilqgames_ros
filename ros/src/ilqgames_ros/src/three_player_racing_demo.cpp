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
#include <ilqgames/constraint/polyline2_signed_distance_constraint.h>
#include <ilqgames/dynamics/concatenated_dynamical_system.h>
#include <ilqgames/dynamics/single_player_car_6d.h>
#include <ilqgames/dynamics/single_player_dubins_car.h>
#include <ilqgames/dynamics/single_player_unicycle_4d.h>
#include <ilqgames/geometry/polyline2.h>
#include <ilqgames/solver/ilq_solver.h>
#include <ilqgames/solver/problem.h>
#include <ilqgames/utils/solver_log.h>
#include <ilqgames/utils/strategy.h>
#include <ilqgames/utils/types.h>
#include <ilqgames/cost/route_progress_cost.h>
#include <ilqgames_ros/three_player_vicon_demo.h>
#include <ilqgames_ros/three_player_racing_demo.h>
#include <ilqgames_ros/racing_lane_center.h>

#include <glog/logging.h>
#include <math.h>
#include <ros/ros.h>
#include <memory>
#include <vector>

namespace ilqgames_ros {

namespace {
// Time.
static constexpr Time kTimeStep = 0.1;      // s
static constexpr Time kTimeHorizon = 100.0;  // s
static constexpr size_t kNumTimeSteps =
    static_cast<size_t>(kTimeHorizon / kTimeStep);

// Flag for one-sided costs.
static constexpr bool kOrientedRight = true;

static constexpr float kP1InitialHeading = M_PI_2;  // rad
static constexpr float kP2InitialHeading = M_PI_2;  // rad
static constexpr float kP3InitialHeading = M_PI_2;  // rad

static constexpr float kP1NominalHeading = M_PI_2;  // rad

// State dimensions.
using P1 = SinglePlayerCar6D;
using P2 = SinglePlayerCar6D;
using P3 = SinglePlayerCar6D;

static const Dimension kP1XIdx = P1::kPxIdx;
static const Dimension kP1YIdx = P1::kPyIdx;
static const Dimension kP1HeadingIdx = P1::kThetaIdx;
static const Dimension kP1PhiIdx = P1::kPhiIdx;
static const Dimension kP1VIdx = P1::kVIdx;
static const Dimension kP1AIdx = P1::kAIdx;

static const Dimension kP2XIdx = P1::kNumXDims + P2::kPxIdx;
static const Dimension kP2YIdx = P1::kNumXDims + P2::kPyIdx;
static const Dimension kP2HeadingIdx = P1::kNumXDims + P2::kThetaIdx;
static const Dimension kP2PhiIdx = P1::kNumXDims + P2::kPhiIdx;
static const Dimension kP2VIdx = P1::kNumXDims + P2::kVIdx;
static const Dimension kP2AIdx = P1::kNumXDims + P2::kAIdx;

// static const Dimension kP2XIdx = P1::kNumXDims + P2::kNumXDims + P2::kPxIdx;
// static const Dimension kP2YIdx = P1::kNumXDims + P2::kNumXDims + P2::kPyIdx;
// static const Dimension kP2HeadingIdx = P1::kNumXDims + P2::kNumXDims +
// P2::kThetaIdx; static const Dimension kP2PhiIdx = P1::kNumXDims +
// P2::kNumXDims + P2::kPhiIdx; static const Dimension kP2VIdx = P1::kNumXDims +
// P2::kNumXDims + P2::kVIdx; static const Dimension kP2AIdx = P1::kNumXDims +
// P2::kNumXDims + P2::kAIdx;

static const Dimension kP3XIdx = P1::kNumXDims + P2::kNumXDims + P3::kPxIdx;
static const Dimension kP3YIdx = P1::kNumXDims + P2::kNumXDims + P3::kPyIdx;
static const Dimension kP3HeadingIdx =
    P1::kNumXDims + P2::kNumXDims + P3::kThetaIdx;
static const Dimension kP3VIdx = P1::kNumXDims + P2::kNumXDims + P3::kVIdx;

}  // anonymous namespace

//might have to add some stuff to this declaration
ThreePlayerRacingDemo::ThreePlayerRacingDemo(const ros::NodeHandle& n)
    : x_idxs_({kP1XIdx, kP2XIdx, kP3XIdx}),
      y_idxs_({kP1YIdx, kP2YIdx, kP3YIdx}),
      heading_idxs_({kP1HeadingIdx, kP2HeadingIdx, kP3HeadingIdx}) {
  LoadParameters(n);

  const std::shared_ptr<const ConcatenatedDynamicalSystem> dynamics(
    new ConcatenatedDynamicalSystem(
       {std::make_shared<SinglePlayerCar6D>(kInterAxleLength),
        std::make_shared<SinglePlayerCar6D>(kInterAxleLength),
        std::make_shared<SinglePlayerCar6D>(kInterAxleLength)},
        kTimeStep));
  /*
  // Create dynamics.
  const std::shared_ptr<const ConcatenatedDynamicalSystem> dynamics(
      new ConcatenatedDynamicalSystem(
          {std::make_shared<P1>(), std::make_shared<P2>(kDubinsV),
           std::make_shared<P3>(kDubinsV)},
          kTimeStep));
  */
  // Set up initial state.
  // NOTE: this will get overwritten before the solver is actually called.
  x0_ = VectorXf::Zero(dynamics->XDim());
  x0_(kP1XIdx) = kP1InitialX;
  x0_(kP1YIdx) = kP1InitialY;
  x0_(kP1HeadingIdx) = kP1InitialHeading;
  x0_(kP1VIdx) = kP1InitialSpeed;
  x0_(kP2XIdx) = kP2InitialX;
  x0_(kP2YIdx) = kP2InitialY;
  x0_(kP2HeadingIdx) = kP2InitialHeading;
  x0_(kP2VIdx) = kP2InitialSpeed;
  x0_(kP3XIdx) = kP3InitialX;
  x0_(kP3YIdx) = kP3InitialY;
  x0_(kP3HeadingIdx) = kP3InitialHeading;
  x0_(kP3VIdx) = kP3InitialSpeed;

  // Set up initial strategies and operating point.
  //how does this differ from not ROS version?
  strategies_.reset(new std::vector<Strategy>());
  for (size_t ii = 0; ii < dynamics->NumPlayers(); ii++)
    strategies_->emplace_back(kNumTimeSteps, dynamics->XDim(),
                              dynamics->UDim(ii));

  operating_point_.reset(
      new OperatingPoint(kNumTimeSteps, dynamics->NumPlayers(), 0.0, dynamics));

  // Set up costs for all players.
  PlayerCost p1_cost, p2_cost, p3_cost;

  // Stay in lanes.
    const PointList2 inner_lane_pts = RacingLaneCenter(
      kP2InitialX, kP2InitialY, side_len, turn_rad_inner, kNumPointsInArc);
    const PointList2 outer_lane_pts = RacingLaneCenter(
      kP1InitialX, kP1InitialY, side_len, turn_rad_outer, kNumPointsInArc);
const PointList2 outermost_lane_pts = RacingLaneCenter(
      kP3InitialX, kP3InitialY, side_len, turn_rad_outermost, kNumPointsInArc);

  const Polyline2 inner_lane(inner_lane_pts);
  const Polyline2 outer_lane(outer_lane_pts);
  const Polyline2 outermost_lane(outermost_lane_pts);

/*
  const std::shared_ptr<QuadraticPolyline2Cost> p1_lane_cost(
      new QuadraticPolyline2Cost(kLaneCostWeight, outer_lane, {kP1XIdx, kP1YIdx},
                                 "LaneCenter"));
  const std::shared_ptr<SemiquadraticPolyline2Cost> p1_lane_r_cost(
      new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, outer_lane,
                                     {kP1XIdx, kP1YIdx}, kLaneHalfWidth,
                                     kOrientedRight, "LaneRightBoundary"));
  const std::shared_ptr<SemiquadraticPolyline2Cost> p1_lane_l_cost(
      new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, outer_lane,
                                     {kP1XIdx, kP1YIdx}, -kLaneHalfWidth,
                                     !kOrientedRight, "LaneLeftBoundary"));
    
  p1_cost.AddStateCost(p1_lane_cost);
  p1_cost.AddStateCost(p1_lane_r_cost);
  p1_cost.AddStateCost(p1_lane_l_cost);
  */
 const std::shared_ptr<Polyline2SignedDistanceConstraint> p1_lane_r_constraint(
      new Polyline2SignedDistanceConstraint(outermost_lane, {kP1XIdx, kP1YIdx},
                                            kLaneHalfWidth, !kOrientedRight,
                                            "LaneRightBoundary"));
  const std::shared_ptr<Polyline2SignedDistanceConstraint> p1_lane_l_constraint(
      new Polyline2SignedDistanceConstraint(inner_lane, {kP1XIdx, kP1YIdx},
                                            -kLaneHalfWidth, kOrientedRight,
                                            "LaneLeftBoundary"));
  p1_cost.AddStateConstraint(p1_lane_r_constraint);
  p1_cost.AddStateConstraint(p1_lane_l_constraint);

  const std::shared_ptr<Polyline2SignedDistanceConstraint> p2_lane_r_constraint(
      new Polyline2SignedDistanceConstraint(outermost_lane, {kP2XIdx, kP2YIdx},
                                            kLaneHalfWidth, !kOrientedRight,
                                            "LaneRightBoundary"));
  const std::shared_ptr<Polyline2SignedDistanceConstraint> p2_lane_l_constraint(
      new Polyline2SignedDistanceConstraint(inner_lane, {kP2XIdx, kP2YIdx},
                                            -kLaneHalfWidth, kOrientedRight,
                                            "LaneLeftBoundary"));
  p2_cost.AddStateConstraint(p2_lane_r_constraint);
  p2_cost.AddStateConstraint(p2_lane_l_constraint);

  const std::shared_ptr<Polyline2SignedDistanceConstraint> p3_lane_r_constraint(
      new Polyline2SignedDistanceConstraint(outermost_lane, {kP3XIdx, kP3YIdx},
                                            kLaneHalfWidth, !kOrientedRight,
                                            "LaneRightBoundary"));
  const std::shared_ptr<Polyline2SignedDistanceConstraint> p3_lane_l_constraint(
      new Polyline2SignedDistanceConstraint(inner_lane, {kP3XIdx, kP3YIdx},
                                            -kLaneHalfWidth, kOrientedRight,
                                            "LaneLeftBoundary"));
  p3_cost.AddStateConstraint(p3_lane_r_constraint);
  p3_cost.AddStateConstraint(p3_lane_l_constraint);

  //progress costs

  // Max/min/nominal speed costs.
  const std::shared_ptr<RouteProgressCost> p1_progress_cost(
      new RouteProgressCost(kP1NominalVCostWeight, kP1NominalV, inner_lane,
                            {kP1XIdx, kP1YIdx}, "RouteProgress"));
  p1_cost.AddStateCost(p1_progress_cost);

  const std::shared_ptr<RouteProgressCost> p2_progress_cost(
      new RouteProgressCost(kP2NominalVCostWeight, kP2NominalV, inner_lane,
                            {kP2XIdx, kP2YIdx}, "RouteProgress"));
  p2_cost.AddStateCost(p2_progress_cost);

  const std::shared_ptr<RouteProgressCost> p3_progress_cost(
      new RouteProgressCost(kP3NominalVCostWeight, kP3NominalV, inner_lane,
                            {kP3XIdx, kP3YIdx}, "RouteProgress"));
  p3_cost.AddStateCost(p3_progress_cost);



    // Max/min/nominal speed costs.
   const auto p1_min_v_cost = std::make_shared<SemiquadraticCost>(
       kMinVCostWeight, kP1VIdx, kMinV, !kOrientedRight, "MinV");
  // const auto p1_max_v_cost = std::make_shared<SemiquadraticCost>(
  //     kMaxVCostWeight, kP1VIdx, kP1MaxV, kOrientedRight, "MaxV");
  const auto p1_nominal_v_cost = std::make_shared<QuadraticCost>(
      kP1NominalVCostWeight, kP1VIdx, kP1NominalV, "NominalV");
p1_cost.AddStateCost(p1_min_v_cost);
  // p1_cost.AddStateCost(p1_max_v_cost);
  p1_cost.AddStateCost(p1_nominal_v_cost);

   const auto p2_min_v_cost = std::make_shared<SemiquadraticCost>(
       kMinVCostWeight, kP2VIdx, kMinV, !kOrientedRight, "MinV");
  // const auto p2_max_v_cost = std::make_shared<SemiquadraticCost>(
  //     kMaxVCostWeight, kP2VIdx, kP2MaxV, kOrientedRight, "MaxV");
  const auto p2_nominal_v_cost = std::make_shared<QuadraticCost>(
      kP2NominalVCostWeight, kP2VIdx, kP2NominalV, "NominalV");
   p2_cost.AddStateCost(p2_min_v_cost);
  // p2_cost.AddStateCost(p2_max_v_cost);
  p2_cost.AddStateCost(p2_nominal_v_cost);

   const auto p3_min_v_cost = std::make_shared<SemiquadraticCost>(
       kMinVCostWeight, kP3VIdx, kMinV, !kOrientedRight, "MinV");
  // const auto p3_max_v_cost = std::make_shared<SemiquadraticCost>(
  //     kMaxVCostWeight, kP3VIdx, kP3MaxV, kOrientedRight, "MaxV");
  const auto p3_nominal_v_cost = std::make_shared<QuadraticCost>(
      kP3NominalVCostWeight, kP3VIdx, kP3NominalV, "NominalV");
   p3_cost.AddStateCost(p3_min_v_cost);
  // p3_cost.AddStateCost(p3_max_v_cost);
  p3_cost.AddStateCost(p3_nominal_v_cost);

  // Curvature costs for P1 and P2.
  // const auto p1_curvature_cost = std::make_shared<QuadraticCost>(
  //     kCurvatureCostWeight, kP1PhiIdx, 0.0, "Curvature");
  // p1_cost.AddStateCost(p1_curvature_cost);

  // const auto p2_curvature_cost = std::make_shared<QuadraticCost>(
  //     kCurvatureCostWeight, kP2PhiIdx, 0.0, "Curvature");
  // p2_cost.AddStateCost(p2_curvature_cost);

  // // Penalize acceleration for cars.
  // const auto p1_a_cost = std::make_shared<QuadraticCost>(kACostWeight,
  // kP1AIdx,
  //                                                        0.0,
  //                                                        "Acceleration");
  // p1_cost.AddStateCost(p1_a_cost);

  // const auto p2_a_cost = std::make_shared<QuadraticCost>(kACostWeight,
  // kP2AIdx,
  //                                                        0.0,
  //                                                        "Acceleration");
  // p2_cost.AddStateCost(p2_a_cost);

  // Penalize control effort.
  const auto p1_omega_cost = std::make_shared<QuadraticCost>(
      kOmegaCostWeight, kP1OmegaIdx, 0.0, "Steering");
  const auto p1_jerk_cost =
      std::make_shared<QuadraticCost>(kJerkCostWeight, kP1JerkIdx, 0.0, "Jerk");
  p1_cost.AddControlCost(0, p1_omega_cost);
  p1_cost.AddControlCost(0, p1_jerk_cost);

  const auto p2_omega_cost = std::make_shared<QuadraticCost>(
      kOmegaCostWeight, kP2OmegaIdx, 0.0, "Steering");
  const auto p2_jerk_cost =
      std::make_shared<QuadraticCost>(kJerkCostWeight, kP2JerkIdx, 0.0, "Jerk");
  p2_cost.AddControlCost(1, p2_omega_cost);
  p2_cost.AddControlCost(1, p2_jerk_cost);

  const auto p3_omega_cost = std::make_shared<QuadraticCost>(
      kOmegaCostWeight, kP3OmegaIdx, 0.0, "Steering");
  const auto p3_a_cost =
      std::make_shared<QuadraticCost>(kJerkCostWeight, kP3JerkIdx, 0.0, "Jerk");
  p3_cost.AddControlCost(2, p3_omega_cost);
  p3_cost.AddControlCost(2, p3_a_cost);

  // Goal costs.
  /*
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

   const auto p3_goalx_cost = std::make_shared<FinalTimeCost>(
       std::make_shared<QuadraticCost>(kGoalCostWeight, kP3XIdx, kP3GoalX),
       kTimeHorizon - kFinalTimeWindow, "GoalX");
   const auto p3_goaly_cost = std::make_shared<FinalTimeCost>(
       std::make_shared<QuadraticCost>(kGoalCostWeight, kP3YIdx, kP3GoalY),
       kTimeHorizon - kFinalTimeWindow, "GoalY");
   p3_cost.AddStateCost(p3_goalx_cost);
   p3_cost.AddStateCost(p3_goaly_cost);
   */

  // Pairwise proximity costs.
   // Pairwise proximity costs.
  const std::shared_ptr<ProximityCost> p1p2_proximity_cost(
      new ProximityCost(kP1ProximityCostWeight, {kP1XIdx, kP1YIdx},
                   {kP2XIdx, kP2YIdx}, kMinProximity, "ProximityP2"));
  const std::shared_ptr<ProximityCost> p1p3_proximity_cost(
      new ProximityCost(kP1ProximityCostWeight, {kP1XIdx, kP1YIdx},
                   {kP3XIdx, kP3YIdx}, kMinProximity, "ProximityP3"));
  p1_cost.AddStateCost(p1p2_proximity_cost);
  p1_cost.AddStateCost(p1p3_proximity_cost);

  const std::shared_ptr<ProximityCost> p2p1_proximity_cost(
      new ProximityCost(kP2ProximityCostWeight, {kP2XIdx, kP2YIdx},
                   {kP1XIdx, kP1YIdx}, kMinProximity, "ProximityP1"));
  const std::shared_ptr<ProximityCost> p2p3_proximity_cost(
      new ProximityCost(kP2ProximityCostWeight, {kP2XIdx, kP2YIdx},
                   {kP3XIdx, kP3YIdx}, kMinProximity, "ProximityP3"));
  p2_cost.AddStateCost(p2p1_proximity_cost);
  p2_cost.AddStateCost(p2p3_proximity_cost);

  const std::shared_ptr<ProximityCost> p3p1_proximity_cost(
      new ProximityCost(kP3ProximityCostWeight, {kP3XIdx, kP3YIdx},
                   {kP1XIdx, kP1YIdx}, kMinProximity, "ProximityP1"));
  const std::shared_ptr<ProximityCost> p3p2_proximity_cost(
      new ProximityCost(kP3ProximityCostWeight, {kP3XIdx, kP3YIdx},
                   {kP2XIdx, kP2YIdx}, kMinProximity, "ProximityP2"));
  //p3_cost.AddStateCost(p3p1_proximity_cost);
  //p3_cost.AddStateCost(p3p2_proximity_cost);

  // Set up solver.
  //params argument not present here, sent over ros network
  solver_.reset(
      new ILQSolver(dynamics, {p1_cost, p2_cost, p3_cost}, kTimeHorizon));
}

void ThreePlayerRacingDemo::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  //goal parameters
  CHECK(nl.getParam("/planner/goals/p1/x", kP1GoalX));
  CHECK(nl.getParam("/planner/goals/p1/y", kP1GoalY));
  CHECK(nl.getParam("/planner/goals/p2/x", kP2GoalX));
  CHECK(nl.getParam("/planner/goals/p2/y", kP2GoalY));
  CHECK(nl.getParam("/planner/goals/p3/x", kP3GoalX));
  CHECK(nl.getParam("/planner/goals/p3/y", kP3GoalY));

  //speed parameters
  CHECK(nl.getParam("/planner/speed/min", kMinV));
  CHECK(nl.getParam("/planner/speed/max", kMaxV));
  CHECK(nl.getParam("/planner/speed/p1Nom", kP1NominalV));
  CHECK(nl.getParam("/planner/speed/p2Nom", kP2NominalV));
  CHECK(nl.getParam("/planner/speed/p3Nom", kP3NominalV));

  //input cost weights
  CHECK(nl.getParam("/planner/weight/u/accel", kACostWeight));
  CHECK(nl.getParam("/planner/weight/u/omega", kOmegaCostWeight));
  CHECK(nl.getParam("/planner/weight/u/jerk", kJerkCostWeight));

  //params on speed of vehicles
  CHECK(nl.getParam("/planner/weight/x/v/max", kMaxVCostWeight));
  CHECK(nl.getParam("/planner/weight/x/v/min", kMinVCostWeight));
  CHECK(nl.getParam("/planner/weight/x/v/p1NomVCost", kP1NominalVCostWeight));
  CHECK(nl.getParam("/planner/weight/x/v/p2NomVCost", kP2NominalVCostWeight));
  CHECK(nl.getParam("/planner/weight/x/v/p3NomVCost", kP3NominalVCostWeight));

 //cost weight of goal
  CHECK(nl.getParam("/planner/weight/x/goal", kGoalCostWeight));

  //lane cost parameters
  CHECK(nl.getParam("/planner/weight/x/lane/center", kLaneCostWeight));
  CHECK(nl.getParam("/planner/weight/x/lane/boundary", kLaneBoundaryCostWeight));
  CHECK(nl.getParam("/planner/lane/half_width", kLaneHalfWidth));

  //proximity cost parameters
  CHECK(nl.getParam("/planner/weight/x/proximity/p1", kP1ProximityCostWeight));
  CHECK(nl.getParam("/planner/weight/x/proximity/p2", kP2ProximityCostWeight));
  CHECK(nl.getParam("/planner/weight/x/proximity/p3", kP3ProximityCostWeight));
  CHECK(nl.getParam("/planner/weight/x/proximity/min", kMinProximity));

  //player 1 initial conditions
  CHECK(nl.getParam("/planner/init/p1x", kP1InitialX));
  CHECK(nl.getParam("/planner/init/p1y", kP1InitialY));
  CHECK(nl.getParam("/planner/init/p1v", kP1InitialSpeed));

  //player 2 initial conditions
  CHECK(nl.getParam("/planner/init/p2x", kP2InitialX));
  CHECK(nl.getParam("/planner/init/p2y", kP2InitialY));
  CHECK(nl.getParam("/planner/init/p2v", kP2InitialSpeed));

  //player 3 initial conditions
  CHECK(nl.getParam("/planner/init/p3x", kP3InitialX));
  CHECK(nl.getParam("/planner/init/p3y", kP3InitialY));
  CHECK(nl.getParam("/planner/init/p3v", kP3InitialSpeed));

  //player 1 control conditions
  CHECK(nl.getParam("/planner/control/p1Omega", kP1OmegaIdx));
  CHECK(nl.getParam("/planner/control/p1Jerk", kP1JerkIdx));

  //player 2 control conditions
  CHECK(nl.getParam("/planner/control/p2Omega", kP2OmegaIdx));
  CHECK(nl.getParam("/planner/control/p2Jerk", kP2JerkIdx));

  //player 3 control conditions
  CHECK(nl.getParam("/planner/control/p3Omega", kP3OmegaIdx));
  CHECK(nl.getParam("/planner/control/p3Jerk", kP3JerkIdx));

  //add track conditions
  CHECK(nl.getParam("/planner/track/inner", turn_rad_inner));
  CHECK(nl.getParam("/planner/track/outer", turn_rad_outer));
  CHECK(nl.getParam("/planner/track/outermost", turn_rad_outermost));
  CHECK(nl.getParam("/planner/track/side", side_len));
  CHECK(nl.getParam("/planner/track/Arcpts", kNumPointsInArc));

  //car parameters
  CHECK(nl.getParam("/planner/car/axle", kInterAxleLength));
}

}  // namespace ilqgames_ros
