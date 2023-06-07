// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#pragma once

#include <nav_msgs/Path.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ros/ros.h>
#include <smug_planner_msgs/GoalSet.h>
#include <visualization_msgs/MarkerArray.h>

#include "smug_planner/validity_type.h"

namespace og = ompl::geometric;
namespace ob = ompl::base;
namespace smug_planner {

class Visualizer {
    ros::NodeHandle nh_;
    ros::Publisher goals_vis_pub_;
    ros::Publisher graph_pub_;
    ros::Publisher invalid_goals_vis_pub_;
    ros::Publisher invalid_pose_vis_pub_;
    ros::Publisher vertex_validity_vis_pub_;
    ros::Publisher edge_validity_vis_pub_;

    visualization_msgs::MarkerArray invalid_goals_marker_array_;
    visualization_msgs::MarkerArray invalid_pose_marker_array_;

    int last_num_vertices{0};
    int last_num_new_vertices{0};
    int last_num_edges{0};
    int last_num_start_goals{0};

    double base_length_;
    double base_width_;
    double base_height_;
    double base_z_offset_;

    int n_no_height_{0};
    int n_collision_{0};
    int n_traversability_low_{0};
    int n_traversability_high_{0};
    int n_valid_{0};

    void addMarkerToArray(visualization_msgs::Marker& marker, visualization_msgs::MarkerArray& marker_array);

   public:
    void visualizeStartAndGoalSets(const geometry_msgs::PoseStamped& start,
                                   const std::vector<smug_planner_msgs::GoalSet>& goal_sets);

    void visualizePlannerGraph(const ob::PlannerData& dat, const std::string& frame_id = "map");

    void visualizeVertexValidity(const std::vector<ob::SE3StateSpace::StateType*> vertex_states,
                                 const std::vector<ValidityType> vertex_validities,
                                 const std::string& frame_id = "map");

    void visualizeEdgeValidity(
        const std::vector<std::pair<ob::SE3StateSpace::StateType*, ob::SE3StateSpace::StateType*>> edge_states,
        const std::vector<ValidityType> edge_validities, const std::string& frame_id = "map");

    void addVertices(const ob::PlannerData& dat, visualization_msgs::MarkerArray& array);

    void addEdges(const ob::PlannerData& dat, visualization_msgs::MarkerArray& array);

    std_msgs::ColorRGBA colorFromValidityType(const ValidityType validity_type, const double a = 1.0);

    std::string nameFromType(const ValidityType validity_type);

    void visualizeInvalidGoals();

    void visualizeInvalidPose();

    void visualizeInvalidGoalAndPose();

    void addInvalidGoalMarker(const geometry_msgs::Point goal, const ValidityType type);

    void addInvalidPoseMarker(const geometry_msgs::PoseStamped pose, const ValidityType type);

    void addInvalidGoalAndPoseMarker(const geometry_msgs::Point goal, const geometry_msgs::PoseStamped pose,
                                     const ValidityType type);

    void clearOutdatedInvalidMarkerAndCount();

    Visualizer(const ros::NodeHandle& nh);
    ~Visualizer();
};

}  // namespace smug_planner
