// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#include "smug_planner_ros/planner_ros.h"

#include <smug_planner/planners/lazy_prm_star_mg.h>  //only used in visualizePlannerGraph()
#include <smug_planner_ros/utils.h>

using namespace smug_planner;

PlannerRos::PlannerRos(const ros::NodeHandle& nh)
    : Planner(loadRosParameters(nh)), nh_(nh), converter_(space_), visualizer_(nh_) {
    ROS_INFO_STREAM("Instanciating PlannerRos object ... ...");
    test_sub_ = nh_.subscribe("toi_config", 1, &PlannerRos::Test_Sub_Callback, this);
    tsdf_sub_ = nh_.subscribe(params_->planner.tsdf_map_topic, 1, &PlannerRos::tsdfMapCallback, this);
    traversability_sub_ =
        nh_.subscribe(params_->planner.traversability_map_topic, 1, &PlannerRos::traversabilityMapCallback, this);
    height_sub_ = nh_.subscribe(params_->planner.height_map_topic, 1, &PlannerRos::heightMapCallback, this);

    toi_sub_ = nh_.subscribe("/tois", 1, &PlannerRos::ToI_Sub_Callback, this);
    pose_sub_ = nh_.subscribe("/robot_pose", 1, &PlannerRos::Pose_Sub_Callback, this);
    dp_path_pub_ = nh_.advertise<nav_msgs::Path>("path_dp", 1, true);
    rba_path_pub_ = nh_.advertise<nav_msgs::Path>("path_rba", 1, true);
    idp_path_pub_ = nh_.advertise<nav_msgs::Path>("path_idp", 1, true);
    path_buffer_pub_ = nh_.advertise<nav_msgs::Path>("path_buffer", 1, true);

    ROS_INFO_STREAM("Creating service server ... ...");

    smug_plan_srv_ = nh_.advertiseService("smug_plan", &PlannerRos::getPlanService, this);
    smug_plan_client_ = nh_.serviceClient<smug_planner_msgs::GetMultiGoalSetPlan>("smug_plan");

    ROS_INFO_STREAM("Instanciate PlannerRos object completed");
}

PlannerRos::~PlannerRos() {}

void PlannerRos::Pose_Sub_Callback(const geometry_msgs::PoseStamped& msg) {
    // check if maps are all set
    if (!tsdf_set_ || !height_set_ || !trav_set_) {
        ROS_WARN("Pose_Sub_Callback : The maps are not there yet");
        return;
    }
    updateMap();
    // check if pose is valid
    ob::ScopedState<StateSpace> start(ss_->getSpaceInformation());
    float z = 0.0;
    start.get()->rotation().w = msg.pose.orientation.w;
    start.get()->rotation().x = msg.pose.orientation.x;
    start.get()->rotation().y = msg.pose.orientation.y;
    start.get()->rotation().z = msg.pose.orientation.z;
    map_->getMapHeightDefaultInf(Eigen::Vector3f(msg.pose.position.x, msg.pose.position.y, 0), &z);
    start.get()->setX(msg.pose.position.x);
    start.get()->setY(msg.pose.position.y);
    start.get()->setZ(z);
    if (checker_->isValid(start)) {
        robot_pose_ = msg;
        robot_pose_.pose.position.z = z;
        valid_robot_pose_set_ = true;
        ROS_INFO("Pose_Sub_Callback : The robot pose is valid");
    } else {
        valid_robot_pose_set_ = false;
        ROS_WARN("Pose_Sub_Callback : The robot pose is invalid, please select somewhere else");
    }
}

void PlannerRos::Test_Sub_Callback(const smug_planner_msgs::GoalConfig& msg) {
    // call the server
    ROS_INFO("Test_Sub_Callback : Start testing request received ");
    waitAndUpdateMap();
    ob::ScopedState<StateSpace> start(ss_->getSpaceInformation());

    ROS_INFO("Test_Sub_Callback : Starting generating goals ... ...");
    std::vector<std::vector<ob::ScopedState<>>> goal_sets;
    if (params_->environment.name == "crater") {
        goal_sets = generateSmallCraterTargetSets(msg.n_set, msg.n_goal_per_set);
        start = generateSmallCraterStart();
    }

    ROS_INFO("Test_Sub_Callback : Finishing genertating goals");

    smug_planner_msgs::GetMultiGoalSetPlan srv;
    srv.request.start = converter_.stateOmplToRos(start);
    int i = 0;  // indicates the start index of every toi
    center_idx_list_.clear();
    for (auto state_set : goal_sets) {
        center_idx_list_.push_back(i);
        i += msg.n_goal_per_set;
        smug_planner_msgs::GoalSet goal_set_msg;
        goal_set_msg.goal_set = converter_.stateOmplToRos(state_set);
        srv.request.goal_sets.push_back(goal_set_msg);
    }
    // visualize goals
    visualizer_.visualizeStartAndGoalSets(srv.request.start, srv.request.goal_sets);

    ROS_INFO("Test_Sub_Callback : Calling service ... ...");
    if (!smug_plan_client_.call(srv)) {
        ROS_INFO("Test_Sub_Callback : Failed to call service smug_plan");
    }
    ROS_INFO("Test_Sub_Callback : End of callback");
    ROS_INFO("=====================================================");
    ROS_INFO("=====================================================");
}

void PlannerRos::ToI_Sub_Callback(const smug_planner_msgs::ToIs& msg) {
    waitAndUpdateMap();

    // first from msg to the input of multiGoalSetPlan
    // start, goal_sets
    // aggregate tois in the goal_sets;
    std::vector<smug_planner_msgs::GoalSet> goal_sets;
    preprocessToIMsgs(msg, goal_sets);

    if (n_invalid_goals_ > 0) {
        if (!params_->mission.plan_with_valid) {
            ROS_INFO("Please reset the mission");
            return;
        } else {
            ROS_INFO("Planning with the rest");
            // planning with the rest valid goals
            // first check if there is any
            if (goal_sets.size() == 0) {
                ROS_INFO("No valid goal in the mission");
                return;
            }
        }
    }

    // get robot pose, here no robot pose is published, only available in simulation
    if (!valid_robot_pose_set_) {
        ROS_WARN("ToI_Sub_Callback : robot start pose not valid, please set or reset a valid start pose");
        ROS_WARN("=====================================================================================");
        ROS_WARN("=====================================================================================");
        return;
    }

    // check again if pose is valid, since the map might have vhanged
    ob::ScopedState<StateSpace> start(ss_->getSpaceInformation());
    float z = 0.0;
    start.get()->rotation().w = robot_pose_.pose.orientation.w;
    start.get()->rotation().x = robot_pose_.pose.orientation.x;
    start.get()->rotation().y = robot_pose_.pose.orientation.y;
    start.get()->rotation().z = robot_pose_.pose.orientation.z;
    map_->getMapHeightDefaultInf(Eigen::Vector3f(robot_pose_.pose.position.x, robot_pose_.pose.position.y, 0), &z);
    start.get()->setX(robot_pose_.pose.position.x);
    start.get()->setY(robot_pose_.pose.position.y);
    start.get()->setZ(z);
    if (checker_->isValid(start)) {
        robot_pose_.pose.position.z = z;
        valid_robot_pose_set_ = true;
        ROS_INFO("ToI_Sub_Callback : The robot pose is valid");
    } else {
        valid_robot_pose_set_ = false;
        ROS_WARN("ToI_Sub_Callback : The robot pose is invalid, please select somewhere else");
        ROS_WARN("=====================================================================================");
        ROS_WARN("=====================================================================================");
        return;
    }

    // need to normalize the quaternion, otherwise ompl throws error
    double qw = robot_pose_.pose.orientation.w;
    double qz = robot_pose_.pose.orientation.z;
    robot_pose_.pose.orientation.w = qw / sqrt(qw * qw + qz * qz);
    robot_pose_.pose.orientation.x = 0.0;
    robot_pose_.pose.orientation.y = 0.0;
    robot_pose_.pose.orientation.z = qz / sqrt(qw * qw + qz * qz);

    smug_planner_msgs::GetMultiGoalSetPlan srv;
    srv.request.start = robot_pose_;
    srv.request.goal_sets = goal_sets;
    // visualize goals
    visualizer_.visualizeStartAndGoalSets(srv.request.start, srv.request.goal_sets);

    ROS_INFO("ToI_Sub_Callback : Calling service ... ...");
    if (!smug_plan_client_.call(srv)) {
        ROS_INFO("ToI_Sub_Callback : Failed to call service smug_plan");
    }
    ROS_INFO("ToI_Sub_Callback : End of callback");
    ROS_INFO("=====================================================");
    ROS_INFO("=====================================================");
}

// format the toi msg into the type compliant to the service GetMultiGoalSetPlan
// and visualize the invalid goals and poses if any
void PlannerRos::preprocessToIMsgs(const smug_planner_msgs::ToIs& msg,
                                   std::vector<smug_planner_msgs::GoalSet>& goal_sets) {
    int i = 0;  // indicates the start index of every toi
    n_invalid_goals_ = 0;
    center_idx_list_.clear();
    for (auto toi : msg.tois) {
        smug_planner_msgs::GoalSet goal_set;

        // handle centroid, put as first element of the goal_set
        geometry_msgs::PoseStamped centroid_pose;
        centroid_pose.pose.position = toi.centroid;

        // need to get a height for the centroid, because the mission publisher uses 2d Nav Goal, which has 0 z value.
        float height;
        if (!map_->getMapHeight(Eigen::Vector3f(toi.centroid.x, toi.centroid.y, 0), &height)) {
            bool has_height = false;
            do {
                // get the height of a point in the vicinity
                has_height =
                    map_->getMapHeight(Eigen::Vector3f(toi.centroid.x + 1 * ((rand() / double(RAND_MAX)) - 0.5),
                                                       toi.centroid.y + 1 * ((rand() / double(RAND_MAX)) - 0.5), 0),
                                       &height);
            } while (!has_height);
        }
        centroid_pose.pose.position.z = height;

        // orientation does not matter
        centroid_pose.pose.orientation.w = 1;
        centroid_pose.pose.orientation.x = 0;
        centroid_pose.pose.orientation.y = 0;
        centroid_pose.pose.orientation.z = 0;
        ob::ScopedState<> centroid_ompl = converter_.poseRosToOmpl(centroid_pose);
        // goal_set.push_back(centroid_ompl);
        goal_set.goal_set.push_back(centroid_pose);

        // add toi coverage to the checker
        double radius = 0;
        int valid_contact_count = 0;
        for (auto contact : toi.contacts) {
            // compute distance from attachment point to center

            double dist_x = contact.x - centroid_pose.pose.position.x;
            double dist_y = contact.y - centroid_pose.pose.position.y;
            radius = std::max(sqrt(dist_x * dist_x + dist_y * dist_y) * 1.1, radius);
            // compute pose from attachment point
            geometry_msgs::PoseStamped attachment_pose;
            if (getAndCheckPoseFromAttachment(toi.centroid, contact, attachment_pose)) {
                goal_set.goal_set.push_back(attachment_pose);
                valid_contact_count++;
            }
        }
        // only add the toi, if it has valid contact
        if (valid_contact_count > 0) {
            checker_validate_toi_->addToI(centroid_ompl, radius);
            inclination_motion_validator_validate_toi_->addToI(centroid_ompl, radius);

            goal_sets.push_back(goal_set);
            center_idx_list_.push_back(i);
            i += valid_contact_count;
        }
    }

    visualizeInvalidGoalAndPose();
}

void PlannerRos::visualizeInvalidGoalAndPose() { visualizer_.visualizeInvalidGoalAndPose(); }

bool PlannerRos::getAndCheckPoseFromAttachment(const geometry_msgs::Point& centroid,
                                               const geometry_msgs::Point& contact, geometry_msgs::PoseStamped& pose) {
    double distance = params_->inspection.distance + params_->robot.torso.length / 2;
    // find x,y, along the ray from centroid to contact
    Eigen::Vector2d v_centroid_contact(contact.x - centroid.x, contact.y - centroid.y);
    double d = v_centroid_contact.norm();
    Eigen::Vector2d v_centroid_pose = v_centroid_contact * (d + distance) / d;
    Eigen::Vector2d pose_xy(v_centroid_pose[0] + centroid.x, v_centroid_pose[1] + centroid.y);
    // get yaw
    double yaw = atan2(-v_centroid_contact[1], -v_centroid_contact[0]);
    // get height

    float height;
    bool has_height = map_->getMapHeightDefaultInf(Eigen::Vector3f(pose_xy[0], pose_xy[1], 0), &height);

    pose.pose.position.x = pose_xy[0];
    pose.pose.position.y = pose_xy[1];
    pose.pose.position.z = height;

    pose.pose.orientation.w = cos(yaw / 2);
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = sin(yaw / 2);

    // check out of bounds
    if (pose.pose.position.x < params_->planner.state_space.x_lim.low ||
        pose.pose.position.x > params_->planner.state_space.x_lim.high ||
        pose.pose.position.y < params_->planner.state_space.y_lim.low ||
        pose.pose.position.y > params_->planner.state_space.y_lim.high) {
        visualizer_.addInvalidGoalAndPoseMarker(contact, pose, ValidityType::COLLISION);
        return false;
    }

    // check pose
    if (!has_height) {
        n_invalid_goals_++;
        visualizer_.addInvalidGoalAndPoseMarker(contact, pose, ValidityType::NO_HEIGHT);
        return false;
    }

    ValidityType type = checkPoseCollision(pose);
    visualizer_.addInvalidGoalAndPoseMarker(contact, pose, type);
    if (type == ValidityType::TRAVERSABILITY_HIGH || type == ValidityType::VALID) {
        return true;
    }

    return false;
}

ValidityType PlannerRos::checkPoseCollision(geometry_msgs::PoseStamped pose) {
    Pose3 state_pose = converter_.poseRosToPose3(pose);
    if (params_->debug.use_traversability) {
        // check traversability
        TraversabilityLevel trav_level = checker_->getTraversabilityLevel(state_pose);
        if (trav_level == HIGH) {
            return ValidityType::TRAVERSABILITY_HIGH;
        }
        if (trav_level == LOW) {
            n_invalid_goals_++;
            return ValidityType::TRAVERSABILITY_LOW;
        }
        // then the level is medium, need to check collision
    }

    if (checker_->bodyNoCollision(state_pose)) {
        return ValidityType::VALID;
    } else {
        n_invalid_goals_++;
        return ValidityType::COLLISION;
    }
}

bool PlannerRos::getPlanService(smug_planner_msgs::GetMultiGoalSetPlan::Request& req,
                                smug_planner_msgs::GetMultiGoalSetPlan::Response& res) {
    ROS_INFO_STREAM("getPlanService : Entered the service");

    // convert ros pose to ompl state
    ob::ScopedState<> start = converter_.poseRosToOmpl(req.start);
    std::vector<std::vector<ob::ScopedState<>>> goal_sets = converter_.poseRosToOmpl(req.goal_sets);
    ROS_INFO("getPlanService : goal_sets size = %ld", goal_sets.size());

    // update map
    waitAndUpdateMap();

    // plan
    ROS_INFO_STREAM("getPlanService : Start main plan");
    multiGoalSetPlan(start, goal_sets);

    // visualize graph
    ROS_INFO("Start visualizing the graph ... ...");
    visualizePlannerGraph();
    if (params_->debug.visualize_vertex_validity_type) {
        ROS_INFO("Start visualizing vertex validities ... ...");
        visualizeVertexValidity();
    }
    if (params_->debug.visualize_vertex_validity_type) {
        ROS_INFO("Start visualizing edge validities ... ...");
        visualizeEdgeValidity();
    }

    // convert path_buffer_ompl_ to path_buffer_ros_
    path_buffer_ros_ = {};
    for (auto path_ompl : path_buffer_ompl_) {
        path_buffer_ros_.push_back(converter_.pathOmplToRos(path_ompl));
    }

    nav_msgs::Path path_ros = converter_.pathOmplToRos(center_complete_path_);
    ROS_INFO_STREAM("getPlanService : Finishing converting ompl path to ros path");

    // publish path_buffer
    ROS_INFO("getPlanService : Publishing path buffer... ...");
    nav_msgs::Path path_buffer;
    path_buffer.header.frame_id = "map";
    // glue together
    for (auto path : path_buffer_ros_) {
        path_buffer.poses.insert(path_buffer.poses.end(), path.poses.begin(), path.poses.end());
    }
    // publish
    path_buffer_pub_.publish(path_buffer);

    nav_msgs::Path dp_path_ros = converter_.pathOmplToRos(complete_path_dp_);
    dp_path_ros.header.frame_id = "map";
    dp_path_pub_.publish(dp_path_ros);

    nav_msgs::Path rba_path_ros = converter_.pathOmplToRos(complete_path_rba_);
    rba_path_ros.header.frame_id = "map";
    rba_path_pub_.publish(rba_path_ros);

    nav_msgs::Path idp_path_ros = converter_.pathOmplToRos(complete_path_idp_);
    idp_path_ros.header.frame_id = "map";
    idp_path_pub_.publish(idp_path_ros);

    return true;
}

void PlannerRos::visualizePlannerGraph() {
    ob::PlannerData dat(ss_->getSpaceInformation());
    ss_->getPlannerData(dat);
    visualizer_.visualizePlannerGraph(dat);
}

void PlannerRos::visualizeVertexValidity() {
    visualizer_.visualizeVertexValidity(planner_->getAllVertexState(), planner_->getAllVertexValidity());
}

void PlannerRos::visualizeEdgeValidity() {
    visualizer_.visualizeEdgeValidity(planner_->getAllEdgeState(), planner_->getAllEdgeValidity());
}

void PlannerRos::tsdfMapCallback(const voxblox_msgs::LayerConstPtr& voxblox_msg) {
    std::lock_guard<std::mutex> lock(tsdf_map_queue_mutex_);
    // ROS_INFO("move new tsdf map to queue... ...");
    voxblox::Layer<voxblox::TsdfVoxel> layer(voxblox_msg->voxel_size, voxblox_msg->voxels_per_side);
    voxblox::deserializeMsgToLayer(*voxblox_msg, &layer);
    tsdf_map_queue_.reset(new voxblox::TsdfMap(layer));
    tsdf_set_ = true;
    // ROS_INFO("queue tsdf map successful");
}

void PlannerRos::traversabilityMapCallback(const voxblox_msgs::LayerConstPtr& voxblox_msg) {
    std::lock_guard<std::mutex> lock(traversability_map_queue_mutex_);
    // ROS_INFO("move traversability map to queue... ...");
    voxblox::Layer<voxblox::TraversabilityVoxel> layer(voxblox_msg->voxel_size, voxblox_msg->voxels_per_side);
    voxblox::deserializeMsgToLayer(*voxblox_msg, &layer);
    traversability_map_queue_.reset(new voxblox::TraversabilityMap(layer));
    trav_set_ = true;
    // ROS_INFO("queue traversability map successful");
}

void PlannerRos::heightMapCallback(const voxblox_msgs::LayerConstPtr& voxblox_msg) {
    std::lock_guard<std::mutex> lock(height_map_queue_mutex_);
    // ROS_INFO("move height map to queue... ...");
    voxblox::Layer<voxblox::HeightVoxel> layer(voxblox_msg->voxel_size, voxblox_msg->voxels_per_side);
    voxblox::deserializeMsgToLayer(*voxblox_msg, &layer);
    height_map_queue_.reset(new voxblox::Layer<voxblox::HeightVoxel>(layer));
    height_set_ = true;
    // ROS_INFO("queue height map successful");
}

// wait until there at least one map of each type was put in the map queue, avoid accessing null ptr map
void PlannerRos::waitAndUpdateMap() {
    if (params_->debug.use_traversability) {
        ros::Rate loopRate(1);
        while (!tsdf_set_ || !height_set_ || !trav_set_) {
            // wait for 1s
            ROS_INFO("wait for maps");
            loopRate.sleep();
        }
    } else {
        ros::Rate loopRate(1);
        while (!tsdf_set_ || !height_set_) {
            // wait for 1s
            ROS_INFO("wait for maps");
            loopRate.sleep();
        }
    }
    updateMap();
}

void PlannerRos::updateMap() {
    updateTsdfMap();
    updateTraversabilityMap();
    updateHeightMap();
}

void PlannerRos::updateTsdfMap() {
    std::lock_guard<std::mutex> lock(tsdf_map_queue_mutex_);
    if (!tsdf_map_queue_) {
        ROS_WARN_STREAM("No new tsdf map received since last planning call.");
    } else {
        ROS_INFO("update tsdf map");
        setMapTsdfMap(std::move(tsdf_map_queue_));
    }
}

void PlannerRos::updateTraversabilityMap() {
    std::lock_guard<std::mutex> lock(traversability_map_queue_mutex_);
    if (!traversability_map_queue_) {
        ROS_WARN_STREAM("No new traversability map received since last planning call.");
    } else {
        ROS_INFO("update traversability map");
        setMapTraversabilityMap(std::move(traversability_map_queue_));
    }
}

void PlannerRos::updateHeightMap() {
    std::lock_guard<std::mutex> lock(height_map_queue_mutex_);
    if (!height_map_queue_) {
        ROS_WARN_STREAM("No new height map received since last planning call.");
    } else {
        ROS_INFO("update height map");
        setMapHeightMap(std::move(height_map_queue_));
    }
}
