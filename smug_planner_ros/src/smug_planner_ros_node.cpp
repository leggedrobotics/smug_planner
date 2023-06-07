// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#include "smug_planner_ros/planner_ros.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "smug_planner");

    ros::NodeHandle nh("~");
    smug_planner::PlannerRos planner(nh);

    ros::AsyncSpinner spinner(nh.param<int>("planner/n_threads", 2));

    spinner.start();
    ros::waitForShutdown();
}
