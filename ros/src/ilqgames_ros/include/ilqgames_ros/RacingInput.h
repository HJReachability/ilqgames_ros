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
// Compute a lane center entering and leaving a roundabout.
//
///////////////////////////////////////////////////////////////////////////////
//#ifdef
//$define

#include <ilqgames/dynamics/concatenated_dynamical_system.h>
#include <ilqgames/solver/top_down_renderable_problem.h>
#include <ilqgames/utils/types.h>

#include <ros/ros.h>
#include <ilqgames_msgs/ThreePlayerRacingInput.h>
#include <ilqgames_msgs/State.h>
#include <ilqgames_msgs/ThreePlayerRacingControl.h>

#include <glog/logging.h>
#include <vector>
#include <memory>

namespace ilqgames {
namespace ilqgames_ros {

class RacingInput {
  public:
    //destructor
    ~RacingInput() {}
    //constructor
    RacingInput(
      const std::shared_ptr<TopDownRenderableProblem>& problem)
      : problem_(problem), initialized_(false), t_(0), t_last_(0) {
    CHECK_NOTNULL(problem.get());
    }
    //initialize
    bool Initialize(const ros::NodeHandle& n);
   
   private:
    //load necessary parameters from rosparam server
    bool LoadParameters(const ros::NodeHandle& n);
    //setup ros subscribers/publishers
    bool RegisterCallbacks(const ros::NodeHandle& n);
    //triggered when timer is iterated every replanning_interval_;
    void TimerCallback(const ros::TimerEvent& e);
    //accepts messages of inputs from the input calculation node
    void ControlCallback(const ilqgames_msgs::ThreePlayerRacingControl::ConstPtr& msg);
    void P1StateCallback(const ilqgames_msgs::State::ConstPtr& msg);
    void P2StateCallback(const ilqgames_msgs::State::ConstPtr& msg);
    void P3StateCallback(const ilqgames_msgs::State::ConstPtr& msg);
    bool ReceivedAllMsgs() const;
    //variable to store the associated racing problem
    std::shared_ptr<TopDownRenderableProblem> problem_;

    //handle trigers to timercallback method every replanning_interval_ sec.
    ros::Timer timer_;
    float control_interval_;
    
    //node name for 
    std::string name_;

    //vars relating to publisher/sub topics
    std::vector<std::string> state_topics_;
    std::string Input_topic_;
    std::string Control_topic_;

    bool initialized_;

    //variables related to sim params
    float kInterAxleLength;
    static constexpr Time kTimeStep = 0.1; 

    //current and last
    double t_;
    double t_last_;

    //list of inputs for n agents
    std::vector<VectorXf> u_;
    std::vector<VectorXf> u_ref_;

    //controller parameters
    std::vector<MatrixXf> P_;
    std::vector<MatrixXf> P_mat_;
    std::vector<VectorXf> alpha_;
    
    //overall system state
    VectorXf x_;
    VectorXf x_ref_;
    VectorXf delta_x_;

     //list of individual system states
    std::vector<VectorXf> x_received_;
  
    //list of publishers for system states
    ros::Publisher Input_pub_;
    ros::Subscriber Control_sub_;
    std::vector<ros::Subscriber> state_subs_;
};

}  // namespace ilqgames 
}  // namespace ilqgames_ros
