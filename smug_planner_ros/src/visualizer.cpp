// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#include "smug_planner_ros/visualizer.h"

#include <smug_planner/planner.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace smug_planner;

Visualizer::Visualizer(const ros::NodeHandle& nh)
    : nh_(nh), base_length_(1.07), base_width_(0.55), base_height_(0.3), base_z_offset_(0.525 + 0.04) {
    goals_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("goals_vis", 1, true);
    graph_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("graph", 1, true);
    invalid_goals_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("invalid_goals_vis", 1, true);
    invalid_pose_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("invalid_pose_vis", 1, true);
    vertex_validity_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("vertex_validity_vis", 1, true);
    edge_validity_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("edge_validity_vis", 1, true);

    nh_.param("robot/torso/length", base_length_, base_length_);
    nh_.param("robot/torso/width", base_width_, base_width_);
    nh_.param("robot/torso/height", base_height_, base_height_);
    double torso_z_offset = 0.04;
    double feet_z_offset = -0.525;
    nh_.param("robot/torso/offset/z", torso_z_offset, torso_z_offset);
    nh_.param("robot/feet/offset/z", feet_z_offset, feet_z_offset);
    base_z_offset_ = torso_z_offset - feet_z_offset;
}

Visualizer::~Visualizer() {}

void Visualizer::addMarkerToArray(visualization_msgs::Marker& marker, visualization_msgs::MarkerArray& marker_array) {
    marker_array.markers.push_back(marker);
}

inline void setMarkerPoseFromState(const ob::State* state, visualization_msgs::Marker& marker) {
    marker.pose.position.x = state->as<Planner::StateType>()->getX();
    marker.pose.position.y = state->as<Planner::StateType>()->getY();
    marker.pose.position.z = state->as<Planner::StateType>()->getZ();
    marker.pose.orientation.w = state->as<Planner::StateType>()->rotation().w;
    marker.pose.orientation.x = state->as<Planner::StateType>()->rotation().x;
    marker.pose.orientation.y = state->as<Planner::StateType>()->rotation().y;
    marker.pose.orientation.z = state->as<Planner::StateType>()->rotation().z;
}

inline void setMarkerPoseFromState(const ob::SE3StateSpace::StateType* state, visualization_msgs::Marker& marker) {
    marker.pose.position.x = state->getX();
    marker.pose.position.y = state->getY();
    marker.pose.position.z = state->getZ();
    marker.pose.orientation.w = state->rotation().w;
    marker.pose.orientation.x = state->rotation().x;
    marker.pose.orientation.y = state->rotation().y;
    marker.pose.orientation.z = state->rotation().z;
}

inline void setMarkerPointFromState(const ob::State* state, geometry_msgs::Point& point) {
    point.x = state->as<Planner::StateType>()->getX();
    point.y = state->as<Planner::StateType>()->getY();
    point.z = state->as<Planner::StateType>()->getZ();
}

inline void setMarkerPointFromStateWithZOffset(const ob::State* state, geometry_msgs::Point& point, double offset) {
    point.x = state->as<Planner::StateType>()->getX();
    point.y = state->as<Planner::StateType>()->getY();
    point.z = state->as<Planner::StateType>()->getZ() + offset;
}

inline void setMarkerPointFromState(const ob::SE3StateSpace::StateType* state, geometry_msgs::Point& point) {
    point.x = state->getX();
    point.y = state->getY();
    point.z = state->getZ();
}

inline void setMarkerPointFromStateWithZOffset(const ob::SE3StateSpace::StateType* state, geometry_msgs::Point& point,
                                               double offset) {
    point.x = state->getX();
    point.y = state->getY();
    point.z = state->getZ() + offset;
}

void Visualizer::visualizeStartAndGoalSets(const geometry_msgs::PoseStamped& start,
                                           const std::vector<smug_planner_msgs::GoalSet>& goal_sets) {
    ROS_INFO_STREAM("Visualizing start and goals ... ...");
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;

    // add start to array
    marker.scale.x = 0.6;
    marker.scale.y = 0.6;
    marker.scale.z = 0.6;
    marker.color.r = 0;
    marker.color.b = 0;
    marker.color.g = 1;
    marker.color.a = 1;
    marker.ns = "start";
    marker.pose = start.pose;
    marker.pose.position.z += 0.5;
    marker_array.markers.push_back(marker);

    // add goals to array

    // marker.ns = "goals";
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.8f;
    marker.color.b = 0;
    marker.color.a = 1;
    int i = 0;
    for (auto goal_set : goal_sets) {
        // marker.ns = std::to_string(i);
        int j = 0;
        for (geometry_msgs::PoseStamped goal : goal_set.goal_set) {
            marker.pose = goal.pose;
            marker.pose.position.z += 0.5;
            if (j == 0) {
                marker.ns = "center";
                marker.scale.x = 1.3;
                marker.scale.y = 1.3;
                marker.scale.z = 1.3;
                marker.color.r = 1;
                marker.color.b = 1;
                marker.color.g = 1;
                marker.color.a = 1;
            } else {
                marker.ns = "pose";
                marker.scale.x = 0.4;
                marker.scale.y = 0.4;
                marker.scale.z = 0.4;
                marker.color.r = 0;
                marker.color.b = 1;
                marker.color.g = 240 / 255.0;
                marker.color.a = 1;
            }
            marker_array.markers.push_back(marker);
            j++;
            ++marker.id;
        }
        i++;
    }

    goals_vis_pub_.publish(marker_array);
}

void Visualizer::visualizePlannerGraph(const ob::PlannerData& dat, const std::string& frame_id) {
    visualization_msgs::MarkerArray array;
    addVertices(dat, array);
    addEdges(dat, array);

    // Set values for all markers.
    const auto time = ros::Time::now();

    for (auto& marker : array.markers) {
        marker.header.stamp = time;
        marker.header.frame_id = frame_id;
        marker.color.a = 1.0;
    }

    graph_pub_.publish(array);
}

void Visualizer::visualizeVertexValidity(const std::vector<ob::SE3StateSpace::StateType*> vertex_states,
                                         const std::vector<ValidityType> vertex_validities,
                                         const std::string& frame_id) {
    visualization_msgs::MarkerArray array;

    visualization_msgs::Marker marker;

    // Do vertices.
    marker.header.frame_id = frame_id;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.08;
    marker.scale.y = 0.035;
    marker.scale.z = 0.008;
    const auto n_vertices = vertex_validities.size();
    for (size_t i = 0; i < n_vertices; ++i) {
        // get vertexValidityProperty_ and set respective color
        ValidityType type = vertex_validities[i];
        marker.color = colorFromValidityType(type, 1);
        marker.ns = "vertex_validity/" + nameFromType(type);
        setMarkerPoseFromState(vertex_states[i], marker);
        array.markers.push_back(marker);
        ++marker.id;
    }

    // const int n_markers = marker.id;
    // marker.action = visualization_msgs::Marker::DELETE;
    // while (marker.id++ < last_num_vertices) {
    //   array.markers.push_back(marker);
    // }
    // last_num_vertices = n_markers;

    vertex_validity_vis_pub_.publish(array);
}

void Visualizer::visualizeEdgeValidity(
    const std::vector<std::pair<ob::SE3StateSpace::StateType*, ob::SE3StateSpace::StateType*>> edge_states,
    const std::vector<ValidityType> edge_validities, const std::string& frame_id) {
    visualization_msgs::MarkerArray array;

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.015;
    marker.scale.y = 0.003;
    marker.scale.z = 0.003;
    marker.points.resize(2);
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;

    const auto n_edges = edge_validities.size();
    for (size_t i = 0; i < n_edges; ++i) {
        ValidityType type = edge_validities[i];
        marker.color = colorFromValidityType(type, 1);
        marker.ns = "edge_validity/" + nameFromType(type);
        auto state = edge_states[i];
        setMarkerPointFromState(state.first, marker.points[0]);
        setMarkerPointFromState(state.second, marker.points[1]);
        array.markers.push_back(marker);
        ++marker.id;
    }

    edge_validity_vis_pub_.publish(array);
}

void Visualizer::addVertices(const ob::PlannerData& dat, visualization_msgs::MarkerArray& array) {
    visualization_msgs::Marker marker;

    // Do vertices.
    marker.ns = "vertices";
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.08;
    marker.scale.y = 0.035;
    marker.scale.z = 0.008;
    marker.color.r = 0.8f;
    const auto n_vertices = dat.numVertices();
    for (size_t i = 0; i < n_vertices; ++i) {
        if (i == last_num_vertices) {
            marker.color.r = 0.8f;
            marker.color.b = 0.0f;
            marker.scale.x = 0.2;
            marker.scale.y = 0.1;
        }

        const auto cur_vert = dat.getVertex(i);
        // og::LazyPRM::Vertex startV = cur_vert;
        // cur_vert.getTag();
        // if(vertexValidityProperty_[og::LazyPRM::vertex_flags_t(cur_vert.getTag())])
        // {

        // }
        setMarkerPoseFromState(cur_vert.getState(), marker);
        array.markers.push_back(marker);
        ++marker.id;
    }

    const int n_markers = marker.id;
    marker.action = visualization_msgs::Marker::DELETE;
    while (marker.id++ < last_num_vertices) {
        array.markers.push_back(marker);
    }
    last_num_vertices = n_markers;
}

void Visualizer::addEdges(const ob::PlannerData& dat, visualization_msgs::MarkerArray& array) {
    visualization_msgs::Marker marker;

    // Do edges.
    marker.ns = "edges";
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.015;
    marker.scale.y = 0.003;
    marker.scale.z = 0.003;
    marker.color.r = 0.8f;
    marker.color.g = 0;
    marker.color.b = 0.0f;
    marker.points.resize(2);
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    std::vector<unsigned int> edges;
    const auto n_vertices = dat.numVertices();
    for (size_t i = 0; i < n_vertices; ++i) {
        const auto cur_vert = dat.getVertex(i);
        setMarkerPointFromState(cur_vert.getState(), marker.points[0]);
        dat.getIncomingEdges(i, edges);
        for (const auto& j : edges) {
            const auto other_vert = dat.getVertex(j);
            setMarkerPointFromState(other_vert.getState(), marker.points[1]);
            array.markers.push_back(marker);
            ++marker.id;
        }
    }

    const int n_markers = marker.id;
    marker.action = visualization_msgs::Marker::DELETE;
    while (marker.id++ < last_num_edges) {
        array.markers.push_back(marker);
    }
    last_num_edges = n_markers;
}

void Visualizer::addInvalidGoalMarker(const geometry_msgs::Point goal, const ValidityType type) {
    double x = goal.x;
    double y = goal.y;
    double z = goal.z;
    if (type == ValidityType::VALID || type == ValidityType::TRAVERSABILITY_HIGH) {
        // return;
    }
    visualization_msgs::Marker marker;
    visualization_msgs::Marker text;

    // arrow marker config
    marker.header.frame_id = "map";
    marker.action = visualization_msgs::Marker::ADD;
    // marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.6;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    // marker.pose.position.z = 0;

    marker.color = colorFromValidityType(type, 1);
    marker.ns = "arrow/" + nameFromType(type);

    // pointing downwards
    marker.pose.orientation.w = sqrt(2) / 2;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = sqrt(2) / 2;
    marker.pose.orientation.z = 0;

    // text marker config
    text.header.frame_id = "map";
    text.action = visualization_msgs::Marker::ADD;
    text.ns = "hint/" + nameFromType(type);
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.scale.z = 0.6;

    text.pose.position.x = x;
    text.pose.position.y = y;

    switch (type) {
        case ValidityType::VALID:
            marker.pose.position.z = 1 + z;
            marker.id = n_valid_;
            text.id = n_valid_;
            text.text = "valid";
            break;
        case ValidityType::NO_HEIGHT:
            marker.pose.position.z = 1;
            marker.id = n_no_height_;
            text.id = n_no_height_;
            text.text = "no height";
            break;
        case ValidityType::COLLISION:
            marker.pose.position.z = 1 + z;
            marker.id = n_collision_;
            text.id = n_collision_;
            text.text = "collision";
            break;
        case ValidityType::TRAVERSABILITY_LOW:
            marker.pose.position.z = 1 + z;
            marker.id = n_traversability_low_;
            text.id = n_traversability_low_;
            text.text = "low trav";
            break;
        case ValidityType::TRAVERSABILITY_HIGH:
            marker.pose.position.z = 1 + z;
            marker.id = n_traversability_high_;
            text.id = n_traversability_high_;
            text.text = "high trav";
            break;
        default:
            break;
    }
    text.pose.position.z = marker.pose.position.z + 0.3;
    text.color = marker.color;

    invalid_goals_marker_array_.markers.push_back(marker);
    invalid_goals_marker_array_.markers.push_back(text);
}

void Visualizer::addInvalidPoseMarker(const geometry_msgs::PoseStamped pose, const ValidityType type) {
    if (type == ValidityType::VALID || type == ValidityType::TRAVERSABILITY_HIGH) {
        // return;
    }

    visualization_msgs::Marker marker;
    visualization_msgs::Marker text;

    // arrow marker config
    marker.header.frame_id = "map";
    marker.action = visualization_msgs::Marker::ADD;
    // marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = base_length_;
    marker.scale.y = base_width_;
    marker.scale.z = base_height_;

    marker.color = colorFromValidityType(type, 0.8);
    marker.ns = nameFromType(type);
    marker.pose = pose.pose;
    marker.pose.position.z = pose.pose.position.z + base_z_offset_;

    switch (type) {
        case ValidityType::VALID:
            marker.id = n_valid_;
            break;

        case ValidityType::NO_HEIGHT:
            marker.pose.position.z = 1 + base_z_offset_;
            marker.id = n_no_height_;
            break;
        case ValidityType::COLLISION:
            marker.id = n_collision_;
            break;
        case ValidityType::TRAVERSABILITY_LOW:
            marker.id = n_traversability_low_;
            break;
        case ValidityType::TRAVERSABILITY_HIGH:
            marker.id = n_traversability_high_;
            break;
        default:
            break;
    }
    invalid_pose_marker_array_.markers.push_back(marker);
}

void Visualizer::addInvalidGoalAndPoseMarker(const geometry_msgs::Point goal, const geometry_msgs::PoseStamped pose,
                                             const ValidityType type) {
    // this part is for keeping track of the marker id
    switch (type) {
        case ValidityType::NO_HEIGHT:
            n_no_height_++;
            break;
        case ValidityType::COLLISION:
            n_collision_++;
            break;
        case ValidityType::TRAVERSABILITY_LOW:
            n_traversability_low_++;
            break;
        case ValidityType::TRAVERSABILITY_HIGH:
            n_traversability_high_++;
            break;
        case ValidityType::VALID:
            n_valid_++;
            break;
        default:
            break;
    }
    addInvalidGoalMarker(goal, type);
    addInvalidPoseMarker(pose, type);
}

void Visualizer::visualizeInvalidGoals() { invalid_goals_vis_pub_.publish(invalid_goals_marker_array_); }

void Visualizer::visualizeInvalidPose() { invalid_pose_vis_pub_.publish(invalid_pose_marker_array_); }

void Visualizer::visualizeInvalidGoalAndPose() {
    visualizeInvalidGoals();
    visualizeInvalidPose();
    clearOutdatedInvalidMarkerAndCount();
}

void Visualizer::clearOutdatedInvalidMarkerAndCount() {
    invalid_goals_marker_array_.markers.clear();
    invalid_pose_marker_array_.markers.clear();
    n_no_height_ = 0;
    n_collision_ = 0;
    n_traversability_low_ = 0;
    n_traversability_high_ = 0;
    n_valid_ = 0;
}

std_msgs::ColorRGBA Visualizer::colorFromValidityType(const ValidityType validity_type, const double a) {
    std_msgs::ColorRGBA color;
    color.a = a;
    switch (validity_type) {
        case ValidityType::UNCHECKED:
            color.r = 1;
            color.g = 1;
            color.b = 0;
            break;

        case ValidityType::VALID:
            color.r = 0;
            color.g = 1;
            color.b = 0;
            break;

        case ValidityType::NO_HEIGHT:
            color.r = 1;
            color.g = 0;
            color.b = 1;
            break;

        case ValidityType::COLLISION:
            color.r = 1;
            color.g = 0.5;
            color.b = 0;
            break;

        case ValidityType::TRAVERSABILITY_LOW:
            color.r = 1;
            color.g = 0;
            color.b = 0;
            break;

        case ValidityType::TRAVERSABILITY_HIGH:
            color.r = 0;
            color.g = 1;
            color.b = 1;
            break;

        case ValidityType::WITHIN_TOI:
            color.r = 0.4;
            color.g = 0;
            color.b = 0.8;
            break;

        default:
            break;
    }
    return color;
}

std::string Visualizer::nameFromType(const ValidityType validity_type) {
    switch (validity_type) {
        case ValidityType::UNCHECKED:
            return "unchecked";
        case ValidityType::VALID:
            return "valid";
        case ValidityType::NO_HEIGHT:
            return "no height";
        case ValidityType::COLLISION:
            return "collision";
        case ValidityType::TRAVERSABILITY_LOW:
            return "low_trav";
        case ValidityType::TRAVERSABILITY_HIGH:
            return "high_trav";
        case ValidityType::WITHIN_TOI:
            return "within_toi";
        default:
            return "-";
    }
}
