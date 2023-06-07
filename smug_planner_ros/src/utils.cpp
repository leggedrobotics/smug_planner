// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#include "smug_planner_ros/utils.h"

using namespace smug_planner;

ParamsPtr smug_planner::loadRosParameters(const ros::NodeHandle& nh) {
    ParamsPtr params = std::make_shared<Params>();

    // Planner.
    params->planner.n_threads = getParamWithDefaultWarning(nh, "planner/n_threads", params->planner.n_threads);
    params->planner.unknown_space_untraversable = getParamWithDefaultWarning(
        nh, "planner/unknown_space_untraversable", params->planner.unknown_space_untraversable);

    params->planner.longest_valid_segment_length = getParamWithDefaultWarning(
        nh, "planner/longest_valid_segment_length", params->planner.longest_valid_segment_length);

    params->planner.overestimate_last_recursion = getParamWithDefaultWarning(
        nh, "planner/overestimate_last_recursion", params->planner.overestimate_last_recursion);

    params->planner.max_stall = getParamWithDefaultWarning(nh, "planner/max_stall", params->planner.max_stall);

    params->planner.traversability_threshold_high = getParamWithDefaultWarning(
        nh, "planner/traversability_threshold_high", params->planner.traversability_threshold_high);
    params->planner.traversability_threshold_low = getParamWithDefaultWarning(
        nh, "planner/traversability_threshold_low", params->planner.traversability_threshold_low);

    params->planner.traversability_offset =
        getParamWithDefaultWarning(nh, "planner/traversability_offset", params->planner.traversability_offset);

    params->planner.tsdf_map_topic =
        getParamWithDefaultWarning(nh, "planner/tsdf_map_topic", params->planner.tsdf_map_topic);

    params->planner.traversability_map_topic =
        getParamWithDefaultWarning(nh, "planner/traversability_map_topic", params->planner.traversability_map_topic);

    params->planner.height_map_topic =
        getParamWithDefaultWarning(nh, "planner/height_map_topic", params->planner.height_map_topic);

    params->planner.state_space.x_lim.low =
        getParamWithDefaultWarning(nh, "planner/state_space/x_lim/low", params->planner.state_space.x_lim.low);

    params->planner.state_space.x_lim.high =
        getParamWithDefaultWarning(nh, "planner/state_space/x_lim/high", params->planner.state_space.x_lim.high);

    params->planner.state_space.y_lim.low =
        getParamWithDefaultWarning(nh, "planner/state_space/y_lim/low", params->planner.state_space.y_lim.low);

    params->planner.state_space.y_lim.high =
        getParamWithDefaultWarning(nh, "planner/state_space/y_lim/high", params->planner.state_space.y_lim.high);

    params->planner.state_space.z_lim.low =
        getParamWithDefaultWarning(nh, "planner/state_space/z_lim/low", params->planner.state_space.z_lim.low);

    params->planner.state_space.z_lim.high =
        getParamWithDefaultWarning(nh, "planner/state_space/z_lim/high", params->planner.state_space.z_lim.high);

    // Motion Validator / Maximum Inclination
    params->motion_validator.max_inclination =
        getParamWithDefaultWarning(nh, "motion_validator/max_inclination", params->motion_validator.max_inclination);

    // Robot.

    params->robot.base_frame = getParamWithDefaultWarning(nh, "robot/base_frame", params->robot.base_frame);

    // Robot / Torso.

    params->robot.torso.length = getParamWithDefaultWarning(nh, "robot/torso/length", params->robot.torso.length);
    params->robot.torso.width = getParamWithDefaultWarning(nh, "robot/torso/width", params->robot.torso.width);
    params->robot.torso.height = getParamWithDefaultWarning(nh, "robot/torso/height", params->robot.torso.height);

    // Robot / Torso / Offset.

    params->robot.torso.offset.x = getParamWithDefaultWarning(nh, "robot/torso/offset/x", params->robot.torso.offset.x);
    params->robot.torso.offset.y = getParamWithDefaultWarning(nh, "robot/torso/offset/y", params->robot.torso.offset.y);
    params->robot.torso.offset.z = getParamWithDefaultWarning(nh, "robot/torso/offset/z", params->robot.torso.offset.z);

    params->robot.torso.collision_checking_tolerance = getParamWithDefaultWarning(
        nh, "robot/torso/collision_checking_tolerance", params->robot.torso.collision_checking_tolerance);

    // Robot / Feet / Offset.
    params->robot.feet.offset.z = getParamWithDefaultWarning(nh, "robot/feet/offset/z", params->robot.feet.offset.z);

    // Inspection.

    params->inspection.distance = getParamWithDefaultWarning(nh, "inspection/distance", params->inspection.distance);

    // Debug
    params->debug.use_traversability =
        getParamWithDefaultWarning(nh, "debug/use_traversability", params->debug.use_traversability);

    params->debug.visualize_vertex_validity_type = getParamWithDefaultWarning(
        nh, "debug/visualize_vertex_validity_type", params->debug.visualize_vertex_validity_type);

    params->debug.visualize_edge_validity_type = getParamWithDefaultWarning(nh, "debug/visualize_edge_validity_type",
                                                                            params->debug.visualize_edge_validity_type);

    // Mission

    params->mission.plan_with_valid =
        getParamWithDefaultWarning(nh, "mission/plan_with_valid", params->mission.plan_with_valid);

    // Benchmark
    params->benchmark.dynamic_programming =
        getParamWithDefaultWarning(nh, "benchmark/dynamic_programming", params->benchmark.dynamic_programming);

    params->benchmark.iterativeDP =
        getParamWithDefaultWarning(nh, "benchmark/iterativeDP", params->benchmark.iterativeDP);

    params->benchmark.rubber_band_algorithm =
        getParamWithDefaultWarning(nh, "benchmark/rubber_band_algorithm", params->benchmark.rubber_band_algorithm);

    params->benchmark.compare = getParamWithDefaultWarning(nh, "benchmark/compare", params->benchmark.compare);

    // Environment
    params->environment.name = getParamWithDefaultWarning(nh, "environment/name", params->environment.name);

    return params;
}
