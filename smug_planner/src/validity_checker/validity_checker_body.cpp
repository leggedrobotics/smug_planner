// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#include "smug_planner/validity_checker/validity_checker_body.h"

#include <ompl/util/Time.h>

using namespace smug_planner;

ValidityCheckerBody::ValidityCheckerBody(const ParamsConstPtr& params) : params_(params) {
    checker3d_.reset(new VoxbloxBoxChecker(
        params_->robot.torso.length, params_->robot.torso.width, params_->robot.torso.height,
        params_->robot.torso.collision_checking_tolerance, params_->planner.unknown_space_untraversable));
}

void ValidityCheckerBody::setMap(const MapPtr& map) { map_ = map; }

static VoxbloxBoxChecker::Pose voxblox_pose;

bool ValidityCheckerBody::isValid(const Pose3& pose) const {
    std::lock_guard<std::mutex> lock(mutex_);

    // Sets the current pose of the robot
    voxblox_pose.pos = pose.translation();
    voxblox_pose.rot = pose.rotation();
    checker3d_->setMap(map_);
    bool valid = !(checker3d_->isInCollision(voxblox_pose));
    return valid;
}

bool ValidityCheckerBody::hasMap() const { return map_->hasMapSet(); }
