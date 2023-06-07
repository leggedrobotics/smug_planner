// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/PathGeometric.h>
#include <smug_planner/planner.h>
#include <smug_planner_msgs/GoalSet.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace smug_planner {

class Converter {
    std::shared_ptr<Planner::StateSpace> space_;

   public:
    Converter(std::shared_ptr<Planner::StateSpace> space);

    Pose3 poseRosToPose3(const geometry_msgs::PoseStamped& pose_ros);

    ob::ScopedState<> poseRosToOmpl(const geometry_msgs::PoseStamped& pose_ros) const;
    std::vector<ob::ScopedState<>> poseRosToOmpl(const std::vector<geometry_msgs::PoseStamped>& pose_ros) const;
    std::vector<std::vector<ob::ScopedState<>>> poseRosToOmpl(
        const std::vector<smug_planner_msgs::GoalSet>& pose_sets_ros) const;

    geometry_msgs::PoseStamped stateOmplToRos(const ob::ScopedState<>& state) const;
    std::vector<geometry_msgs::PoseStamped> stateOmplToRos(const std::vector<ob::ScopedState<>>& states) const;

    nav_msgs::Path pathOmplToRos(const og::PathGeometric& path) const;
    nav_msgs::Path pathOmplToRos(const og::PathGeometricPtr& path_ompl) const;

    std::vector<std::vector<double>> ratingRosToVector(
        const std::vector<smug_planner_msgs::GoalSet>& pose_sets_ros) const;
};

}  // namespace smug_planner
