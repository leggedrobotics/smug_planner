// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#include "smug_planner_ros/converter.h"

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace smug_planner;

Converter::Converter(std::shared_ptr<Planner::StateSpace> space) : space_(space) {}

Pose3 Converter::poseRosToPose3(const geometry_msgs::PoseStamped& pose_ros) {
    Pose3 pose;
    pose.translation().x() = pose_ros.pose.position.x;
    pose.translation().y() = pose_ros.pose.position.y;
    pose.translation().z() = pose_ros.pose.position.z;
    pose.matrix().topLeftCorner(3, 3) =
        Eigen::Quaternion<Scalar>(pose_ros.pose.orientation.w, pose_ros.pose.orientation.x, pose_ros.pose.orientation.y,
                                  pose_ros.pose.orientation.z)
            .toRotationMatrix();
    return pose;
}

ob::ScopedState<> Converter::poseRosToOmpl(const geometry_msgs::PoseStamped& pose_ros) const {
    ob::ScopedState<> pose(space_);

    auto pose_se3 = pose->as<Planner::StateType>();
    pose_se3->setX(pose_ros.pose.position.x);
    pose_se3->setY(pose_ros.pose.position.y);
    pose_se3->setZ(pose_ros.pose.position.z);
    pose_se3->rotation().w = pose_ros.pose.orientation.w;
    pose_se3->rotation().x = pose_ros.pose.orientation.x;
    pose_se3->rotation().y = pose_ros.pose.orientation.y;
    pose_se3->rotation().z = pose_ros.pose.orientation.z;

    return pose;
}

geometry_msgs::PoseStamped Converter::stateOmplToRos(const ob::ScopedState<>& state) const {
    geometry_msgs::PoseStamped pose_ros;
    const auto state_ompl = state->as<Planner::StateType>();
    // from se2 state to se3 ros pose
    pose_ros.pose.position.x = state_ompl->getX();
    pose_ros.pose.position.y = state_ompl->getY();
    pose_ros.pose.position.z = state_ompl->getZ();
    pose_ros.pose.orientation.w = state_ompl->rotation().w;
    pose_ros.pose.orientation.x = state_ompl->rotation().x;
    pose_ros.pose.orientation.y = state_ompl->rotation().y;
    pose_ros.pose.orientation.z = state_ompl->rotation().z;
    return pose_ros;
}

std::vector<geometry_msgs::PoseStamped> Converter::stateOmplToRos(const std::vector<ob::ScopedState<>>& states) const {
    std::vector<geometry_msgs::PoseStamped> poses;
    for (auto state : states) {
        poses.push_back(stateOmplToRos(state));
    }
    return poses;
}

std::vector<ob::ScopedState<>> Converter::poseRosToOmpl(const std::vector<geometry_msgs::PoseStamped>& pose_ros) const {
    std::vector<ob::ScopedState<>> poses;
    for (auto pose : pose_ros) {
        poses.push_back(poseRosToOmpl(pose));
    }
    return poses;
}

std::vector<std::vector<ob::ScopedState<>>> Converter::poseRosToOmpl(
    const std::vector<smug_planner_msgs::GoalSet>& pose_sets_ros) const {
    std::vector<std::vector<ob::ScopedState<>>> state_sets;
    for (auto pose_set : pose_sets_ros) {
        state_sets.push_back(poseRosToOmpl(pose_set.goal_set));
    }
    return state_sets;
}

nav_msgs::Path Converter::pathOmplToRos(const og::PathGeometric& path_ompl) const {
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.poses.resize(path_ompl.getStateCount());
    for (size_t i = 0; i < path_ompl.getStateCount(); ++i) {
        const auto state = path_ompl.getState(i);
        const auto state_ompl = state->as<Planner::StateType>();
        path.poses[i].header.frame_id = "map";
        path.poses[i].pose.position.x = state_ompl->getX();
        path.poses[i].pose.position.y = state_ompl->getY();
        path.poses[i].pose.position.z = state_ompl->getZ();
        path.poses[i].pose.orientation.x = state_ompl->rotation().x;
        path.poses[i].pose.orientation.y = state_ompl->rotation().y;
        path.poses[i].pose.orientation.z = state_ompl->rotation().z;
        path.poses[i].pose.orientation.w = state_ompl->rotation().w;
    }

    return path;
}

nav_msgs::Path Converter::pathOmplToRos(const og::PathGeometricPtr& path_ompl) const {
    return pathOmplToRos(*path_ompl);
}
