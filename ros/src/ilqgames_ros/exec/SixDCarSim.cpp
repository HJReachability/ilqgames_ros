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

#include <ilqgames_ros/receding_horizon_planner.h>
#include <ilqgames_ros/three_player_racing_demo.h>
#include <ilqgames_ros/SixDCarSim.h>

#include <glog/logging.h>
#include <ros/ros.h>
#include <memory>

int main(int argc, char** argv) {
  const std::string log_file =
      ILQGAMES_ROS_LOG_DIR + std::string("/state writer.log");
  google::SetLogDestination(0, log_file.c_str());
  google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "ilqgame_planner");
  ros::NodeHandle n("~");

  auto problem = std::make_shared<ilqgames_ros::ThreePlayerRacingDemo>(n);
  ilqgames::ilqgames_ros::SixDCarSim planner(problem);

  if (!planner.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize Sim.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  ros::spin();

  return EXIT_SUCCESS;
}
