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
#include <ilqgames/dynamics/single_player_car_6d.h>
#include <ilqgames/dynamics/single_player_dubins_car.h>
#include <ilqgames/dynamics/single_player_unicycle_4d.h>
#include <ilqgames/examples/racing_lane_center.h>
#include <ilqgames/geometry/polyline2.h>
#include <ilqgames/solver/ilq_solver.h>
#include <ilqgames/solver/problem.h>
#include <ilqgames/utils/solver_log.h>
#include <ilqgames/utils/strategy.h>
#include <ilqgames/utils/types.h>
#include <ilqgames_ros/three_player_vicon_demo.h>
#include <ilqgames_ros/three_player_racing_demo.h>

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

//comment here to end of namespace to correct server
/*
//is this needed here?
static constexpr float kInterAxleLength = 4.0;  // m

// Cost weights.
static constexpr float kOmegaCostWeight = 500000.0;
static constexpr float kJerkCostWeight = 500.0;

static constexpr float kACostWeight = 50.0;
static constexpr float kP1NominalVCostWeight = 10.0;
static constexpr float kP2NominalVCostWeight = 1.0;
static constexpr float kP3NominalVCostWeight = 1.0;

static constexpr float kMinVCostWeight = 10.0;

static constexpr float kLaneCostWeight = 25.0; 
static constexpr float kLaneBoundaryCostWeight = 100.0; 

static constexpr float kMinProximity = 5.0;
static constexpr float kP1ProximityCostWeight = 100.0;
static constexpr float kP2ProximityCostWeight = 100.0;
static constexpr float kP3ProximityCostWeight = 100.0;
using ProxCost = ProximityCost;

// Heading weight
static constexpr float kNominalHeadingCostWeight = 150.0;

// Lane width.
static constexpr float kLaneHalfWidth = 2.5;  // m

// Nominal speed.
static constexpr float kP1NominalV = 15.0;  // m/s
static constexpr float kP2NominalV = 10.0;  // m/s
static constexpr float kP3NominalV = 10.0;  // m/s

static constexpr float kMinV = 2.0;  // m/s

// Initial state.
static constexpr float kP1InitialX = 2.5;    // m
static constexpr float kP1InitialY = -10.0;  // m

static constexpr float kP2InitialX = -1.0;   // m
static constexpr float kP2InitialY = -10.0;  // m

static constexpr float kP3InitialX = 2.5;   // m
static constexpr float kP3InitialY = 10.0;  // m

//set goal points 

//static constexpr float kP1GoalX = kP1InitialX - 100 ;   // m
//static constexpr float kP1GoalY = 18.0;  // m

//static constexpr float kP2GoalX = kP2InitialX - 100;  // m
//static constexpr float kP2GoalY = 18.0;   // m

//static constexpr float kP3GoalX = kP3InitialX - 100;  // m
//static constexpr float kP3GoalY = 21.5;   // m

//goal weight
//static constexpr float kGoalCostWeight = 10000;

static constexpr float kP1InitialSpeed = 10.0;  // m/s
static constexpr float kP2InitialSpeed = 2.0;   // m/s
static constexpr float kP3InitialSpeed = 2.0;   // m/s

// Control dimensions.
static const Dimension kP1OmegaIdx = 0;
static const Dimension kP1JerkIdx = 1;
static const Dimension kP2OmegaIdx = 0;
static const Dimension kP2JerkIdx = 1;
static const Dimension kP3OmegaIdx = 0;
static const Dimension kP3JerkIdx = 1;

//track parameters
static constexpr float turn_rad_inner = 15.0;
static constexpr float turn_rad_outer = 20.0;
static constexpr float side_len = 25.0;
static constexpr float kNumPointsInArc = 5;
*/
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
  x0_ = VectorXf::Constant(dynamics->XDim(),
                           std::numeric_limits<float>::quiet_NaN());

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
      kP1InitialX, kP1InitialY, side_len, turn_rad_outer, kNumPointsInArc);;

  const Polyline2 inner_lane(inner_lane_pts);
  const Polyline2 outer_lane(outer_lane_pts);

  const std::shared_ptr<QuadraticPolyline2Cost> p1_lane_cost(
      new QuadraticPolyline2Cost(kLaneCostWeight, inner_lane, {kP1XIdx, kP1YIdx},
                                 "LaneCenter"));
  const std::shared_ptr<SemiquadraticPolyline2Cost> p1_lane_r_cost(
      new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, inner_lane,
                                     {kP1XIdx, kP1YIdx}, kLaneHalfWidth,
                                     kOrientedRight, "LaneRightBoundary"));
  const std::shared_ptr<SemiquadraticPolyline2Cost> p1_lane_l_cost(
      new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, inner_lane,
                                     {kP1XIdx, kP1YIdx}, -kLaneHalfWidth,
                                     !kOrientedRight, "LaneLeftBoundary"));
  p1_cost.AddStateCost(p1_lane_cost);
  p1_cost.AddStateCost(p1_lane_r_cost);
  p1_cost.AddStateCost(p1_lane_l_cost);

  const std::shared_ptr<QuadraticPolyline2Cost> p2_lane_cost(
      new QuadraticPolyline2Cost(kLaneCostWeight, inner_lane, {kP2XIdx, kP2YIdx},
                                 "LaneCenter"));
  const std::shared_ptr<SemiquadraticPolyline2Cost> p2_lane_r_cost(
      new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, inner_lane,
                                     {kP2XIdx, kP2YIdx}, kLaneHalfWidth,
                                     kOrientedRight, "LaneRightBoundary"));
  const std::shared_ptr<SemiquadraticPolyline2Cost> p2_lane_l_cost(
      new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, inner_lane,
                                     {kP2XIdx, kP2YIdx}, -kLaneHalfWidth,
                                     !kOrientedRight, "LaneLeftBoundary"));
  p2_cost.AddStateCost(p2_lane_cost);
  p2_cost.AddStateCost(p2_lane_r_cost);
  p2_cost.AddStateCost(p2_lane_l_cost);

  const std::shared_ptr<QuadraticPolyline2Cost> p3_lane_cost(
      new QuadraticPolyline2Cost(kLaneCostWeight, outer_lane, {kP3XIdx, kP3YIdx},
                                 "LaneCenter"));
  const std::shared_ptr<SemiquadraticPolyline2Cost> p3_lane_r_cost(
      new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, outer_lane,
                                     {kP3XIdx, kP3YIdx}, kLaneHalfWidth,
                                     kOrientedRight, "LaneRightBoundary"));
  const std::shared_ptr<SemiquadraticPolyline2Cost> p3_lane_l_cost(
      new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, outer_lane,
                                     {kP3XIdx, kP3YIdx}, -kLaneHalfWidth,
                                     !kOrientedRight, "LaneLeftBoundary"));
  p3_cost.AddStateCost(p3_lane_cost);
  p3_cost.AddStateCost(p3_lane_r_cost);
  p3_cost.AddStateCost(p3_lane_l_cost);

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
  //p3_cost.AddStateCost(p3p1_proximity_cost)
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
  CHECK(nl.getParam("/planner/track/side", side_len));
  CHECK(nl.getParam("/planner/track/Arcpts", kNumPointsInArc));

  //car parameters
  CHECK(nl.getParam("/planner/car/axle", kInterAxleLength));
}

}  // namespace ilqgames_ros
