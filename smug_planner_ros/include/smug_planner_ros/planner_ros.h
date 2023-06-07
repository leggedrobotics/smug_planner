// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#pragma once

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/GetPlan.h>
#include <ros/ros.h>
#include <smug_planner_msgs/GoalConfig.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <tf2_ros/transform_listener.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox_msgs/Layer.h>
#include <voxblox_ros/conversions.h>

#include <thread>

#include "smug_planner/planner.h"
#include "smug_planner_msgs/GetMultiGoalSetPlan.h"
#include "smug_planner_msgs/GoalSet.h"
#include "smug_planner_msgs/ToIs.h"
#include "smug_planner_ros/converter.h"
#include "smug_planner_ros/visualizer.h"

namespace smug_planner {

class PlannerRos : protected Planner {
   protected:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_{tf_buffer_};

    ros::Subscriber tsdf_sub_;
    ros::Subscriber height_sub_;
    ros::Subscriber traversability_sub_;

    mutable std::mutex tsdf_map_queue_mutex_;
    TsdfMapPtr tsdf_map_queue_;
    bool tsdf_set_{false};

    mutable std::mutex height_map_queue_mutex_;
    HeightMapPtr height_map_queue_;
    bool height_set_{false};

    mutable std::mutex traversability_map_queue_mutex_;
    TraversabilityMapPtr traversability_map_queue_;
    bool trav_set_{false};

    // flag indicate if a map is requested
    bool request_map_{false};

    ros::Subscriber toi_sub_;
    ros::Subscriber test_sub_;
    ros::Subscriber pose_sub_;

    ros::Publisher dp_path_pub_;
    ros::Publisher rba_path_pub_;
    ros::Publisher idp_path_pub_;
    ros::Publisher path_buffer_pub_;

    std::vector<nav_msgs::Path> path_buffer_ros_;

    ros::ServiceServer smug_plan_srv_;

    ros::ServiceClient smug_plan_client_;

    geometry_msgs::PoseStamped robot_pose_;
    bool valid_robot_pose_set_{false};

    Converter converter_;
    Visualizer visualizer_;

    int n_invalid_goals_{0};

    void tsdfMapCallback(const voxblox_msgs::LayerConstPtr& voxblox_msg);

    void traversabilityMapCallback(const voxblox_msgs::LayerConstPtr& voxblox_msg);

    void heightMapCallback(const voxblox_msgs::LayerConstPtr& voxblox_msg);

    void waitAndUpdateMap();

    void updateMap();

    void updateTsdfMap();

    void updateTraversabilityMap();

    void updateHeightMap();

    virtual void ToI_Sub_Callback(const smug_planner_msgs::ToIs& msg);

    void Pose_Sub_Callback(const geometry_msgs::PoseStamped& msg);

    void preprocessToIMsgs(const smug_planner_msgs::ToIs& msg, std::vector<smug_planner_msgs::GoalSet>& goal_sets);

    bool getAndCheckPoseFromAttachment(const geometry_msgs::Point& centroid, const geometry_msgs::Point& attachment,
                                       geometry_msgs::PoseStamped& pose);

    ValidityType checkPoseCollision(geometry_msgs::PoseStamped pose);

    virtual void Test_Sub_Callback(const smug_planner_msgs::GoalConfig& msg);

    bool getPlanService(smug_planner_msgs::GetMultiGoalSetPlan::Request& req,
                        smug_planner_msgs::GetMultiGoalSetPlan::Response& res);

    bool transformRosPoseToMapFrame(const geometry_msgs::PoseStamped& in,
                                    geometry_msgs::PoseStamped& out) const;  // not used now

    void visualizePlannerGraph();

    void visualizeVertexValidity();

    void visualizeEdgeValidity();

    void visualizeInvalidGoalAndPose();

   public:
    ~PlannerRos();
    explicit PlannerRos(const ros::NodeHandle& nh);
};

}  // namespace smug_planner
