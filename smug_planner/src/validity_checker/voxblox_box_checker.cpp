// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#include "smug_planner/validity_checker/voxblox_box_checker.h"

#include <ompl/util/Time.h>

using namespace smug_planner;

VoxbloxBoxChecker::VoxbloxBoxChecker(float length_x, float length_y, float length_z, float tolerance,
                                     bool unknown_space_collision) {
    main_box_.length = length_x;
    main_box_.width = length_y;
    main_box_.height = length_z;

    tolerance_ = tolerance;
    unknown_space_collision_ = unknown_space_collision;
}

bool VoxbloxBoxChecker::isInCollision(const Pose &main_pose) const {
    return isInCollision(main_pose, main_box_, main_pose);
}

bool VoxbloxBoxChecker::isInCollision(const Pose &pose, const Box &box, const Pose &main_pose) const {
    // distance to the nearest obstacle
    float distance;

    // if no map information is available at the current position, we return
    bool get_dist_success = getDistance(pose, &distance);

    bool deepest_recursion = std::max({box.length, box.width, box.height}) < tolerance_;

    if (!get_dist_success) {
    } else {
        float room_diagonal = sqrtf(box.height * box.height + box.length * box.length + box.width * box.width);

        // sphere that wraps around box - if no obstacle within, no collision
        if (distance > room_diagonal * 0.5) {
            return false;
        } else if (deepest_recursion) {
            return true;
        }

        // sphere that completely lies inside box - if obstacle within, definite collision
        if (distance < std::min({box.length, box.width, box.height}) * 0.5) {
            return true;
        }
    }

    // make sure the calculations converge at some point
    if (deepest_recursion) {
        // this block is only entered if !getDistance in the deepest recursion
        return unknown_space_collision_;
    }

    // if no collision detected so far, divide into two boxes along longest side
    Box subbox1 = box;
    Box subbox2 = box;
    Pose subpose1 = pose;
    Pose subpose2 = pose;

    createSubposesAndSubboxes(pose, &subpose1, &subpose2, box, &subbox1, &subbox2);
    // recursive call to check the subboxes
    return isInCollision(subpose1, subbox1, main_pose) || isInCollision(subpose2, subbox2, main_pose);
}

void VoxbloxBoxChecker::createSubposesAndSubboxes(const Pose &pose, Pose *subpose1, Pose *subpose2, const Box &box,
                                                  Box *subbox1, Box *subbox2) {
    // check x-direction first: for body, it is the longest side
    if (box.length > box.width && box.length > box.height) {
        subbox1->length *= 0.5;
        subbox2->length *= 0.5;

        // creates the new position points for the subposes
        // the orientations are left the same
        subpose1->pos += subbox1->length * 0.5 * pose.rot.col(0);
        subpose2->pos -= subbox2->length * 0.5 * pose.rot.col(0);

        return;
    }

    if (box.width > box.height) {
        subbox1->width *= 0.5;
        subbox2->width *= 0.5;

        subpose1->pos += subbox1->width * 0.5 * pose.rot.col(1);
        subpose2->pos -= subbox2->width * 0.5 * pose.rot.col(1);

        return;
    }

    subbox1->height *= 0.5;
    subbox2->height *= 0.5;

    subpose1->pos += subbox1->height * 0.5 * pose.rot.col(2);
    subpose2->pos -= subbox2->height * 0.5 * pose.rot.col(2);
}

void VoxbloxBoxChecker::setMap(std::shared_ptr<Map> map) {
    std::lock_guard<std::mutex> lock(mutex_);
    map_ = map;
}

bool VoxbloxBoxChecker::getDistance(const Pose &pose, float *distance) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_->getMapDistance3D(pose.pos, distance);
}
