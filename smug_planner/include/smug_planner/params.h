// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#pragma once

#include <cmath>
#include <memory>
#include <string>

namespace smug_planner {

// For parameter description see "params.yaml".

// need to clean the params
struct Params {
    struct {
        unsigned int n_threads{1};
        bool unknown_space_untraversable{false};

        double longest_valid_segment_length{0.5};

        bool overestimate_last_recursion{false};

        int max_stall{3};

        double traversability_threshold_high{0.95};
        double traversability_threshold_low{0.1};

        double traversability_offset{0.01};

        std::string tsdf_map_topic{"/voxblox_node/tsdf_map_out"};
        std::string traversability_map_topic{"/voxblox_node/traversability_layer"};
        std::string height_map_topic{"/voxblox_node/raw_height_layer"};

        struct {
            struct {
                double low{-17.0};
                double high{17.0};
            } x_lim;

            struct {
                double low{-17.0};
                double high{17.0};
            } y_lim;

            struct {
                double low{-5.0};
                double high{5.0};
            } z_lim;
        } state_space;

    } planner;

    struct {
        double max_inclination{0.58};
    } motion_validator;

    struct {
        std::string base_frame{"base"};

        struct {
            double length{1.05};
            double width{0.55};
            double height{0.2};

            struct {
                double x{0.0};
                double y{0.0};
                double z{0.0};
            } offset;

            double collision_checking_tolerance{0.01};

        } torso;

        struct {
            struct {
                double z{-0.525};
            } offset;

        } feet;

    } robot;

    struct {
        double distance{0.2};
    } inspection;

    struct {
        bool use_traversability{true};
        bool visualize_vertex_validity_type{true};
        bool visualize_edge_validity_type{true};
    } debug;

    struct {
        bool plan_with_valid{false};
    } mission;

    struct {
        bool dynamic_programming{true};
        bool iterativeDP{true};
        bool rubber_band_algorithm{false};
        bool compare{false};
    } benchmark;

    struct {
        std::string name{"crater"};
    } environment;
};

using ParamsPtr = std::shared_ptr<Params>;
using ParamsConstPtr = std::shared_ptr<const Params>;

}  // namespace smug_planner
