// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include "smug_planner/map/map.h"
#include "smug_planner/utils.h"

namespace smug_planner {

class VoxbloxBoxChecker {
   public:
    struct Pose {
        Eigen::Vector3f pos;
        Eigen::Matrix3f rot;
    };

   private:
    mutable std::mutex mutex_;

    std::shared_ptr<Map> map_;

    struct Box {
        float height;
        float width;
        float length;
    } main_box_{};

    // determines the smallest box size until recursion stops
    float tolerance_;

    // determines whether unknown space is in collision
    bool unknown_space_collision_;

   public:
    VoxbloxBoxChecker(float length_x, float length_y, float length_z, float tolerance, bool unknown_space_collision);

    // returns true, if box collides with obstacle, false if it doesn't collide
    bool isInCollision(const Pose &main_pose) const;

    void setMap(std::shared_ptr<Map> map);

   private:
    bool isInCollision(const Pose &pose, const Box &box, const Pose &main_pose) const;

    // distance to the closest obstacle
    bool getDistance(const Pose &pose, float *distance) const;

    // sets the subposes and subboxes for the recursive collision checking
    static void createSubposesAndSubboxes(const Pose &pose, Pose *subpose1, Pose *subpose2, const Box &box,
                                          Box *subbox1, Box *subbox2);
};

}  // namespace smug_planner