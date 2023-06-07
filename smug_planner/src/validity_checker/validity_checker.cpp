// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#include "smug_planner/validity_checker/validity_checker.h"

#include <ompl/util/Time.h>

using namespace smug_planner;

StateValidityChecker::StateValidityChecker(const ob::SpaceInformationPtr &si, const ParamsConstPtr &params)
    : ob::StateValidityChecker(si), checker_body_(params), params_(params) {}

void StateValidityChecker::setMap(const std::shared_ptr<Map> &map) {
    map_ = map;
    checker_body_.setMap(map);
}

bool StateValidityChecker::hasMap() const { return map_->hasMapSet() && checker_body_.hasMap(); }

bool StateValidityChecker::isValid(const ob::State *state) const {
    const auto state_pose = Pose3FromSE3(state);

    // check if has height
    const auto state_translation = state_pose.translation();

    if (state_translation.z() == INFINITY) {
        return false;
    }

    if (params_->debug.use_traversability) {
        // check traversability
        TraversabilityLevel trav_level = getTraversabilityLevel(state_pose);

        if (trav_level == HIGH) {
            // do not perform collision checking
            return true;
        }
        if (trav_level == LOW) {
            // definitely collision
            return false;
        }

        // then the level is medium, need to check collision
    }

    const auto pose_base = state_pose * Pose3FromXYZ(params_->robot.torso.offset.x, params_->robot.torso.offset.y,
                                                     params_->robot.torso.offset.z - params_->robot.feet.offset.z);
    bool valid = checker_body_.isValid(pose_base);
    return valid;
}

ValidityType StateValidityChecker::validityType(const ob::State *state) const {
    const auto state_pose = Pose3FromSE3(state);

    // check if has height
    const auto state_translation = state_pose.translation();

    if (state_translation.z() == INFINITY) {
        return ValidityType::NO_HEIGHT;
    }

    if (params_->debug.use_traversability) {
        // check traversability
        TraversabilityLevel trav_level = getTraversabilityLevel(state_pose);
        if (trav_level == HIGH) {
            // do not perform collision checking
            return ValidityType::TRAVERSABILITY_HIGH;
        }
        if (trav_level == LOW) {
            // definitely collision
            return ValidityType::TRAVERSABILITY_LOW;
        }

        // then the level is medium, need to check collision
    }

    const auto pose_base = state_pose * Pose3FromXYZ(params_->robot.torso.offset.x, params_->robot.torso.offset.y,
                                                     params_->robot.torso.offset.z - params_->robot.feet.offset.z);
    bool valid = checker_body_.isValid(pose_base);
    if (valid) return ValidityType::VALID;
    return ValidityType::COLLISION;
}

// input state pose
bool StateValidityChecker::bodyNoCollision(const Pose3 &pose) const {
    const auto pose_base = pose * Pose3FromXYZ(params_->robot.torso.offset.x, params_->robot.torso.offset.y,
                                               params_->robot.torso.offset.z - params_->robot.feet.offset.z);

    return checker_body_.isValid(pose_base);
}

// input is state pose
TraversabilityLevel StateValidityChecker::getTraversabilityLevel(const Pose3 &pose) const {
    Eigen::Vector3f position = pose.translation();
    // add offset of the traversability map to the height map
    position[2] += params_->planner.traversability_offset;
    float traversability;
    if (!map_->getMapTraversability(position, &traversability)) {
        // directly invalid because no traversability
        return LOW;
    }
    if (traversability > params_->planner.traversability_threshold_high) {
        // directly valid because traversability high enough
        return HIGH;
    }
    if (traversability < params_->planner.traversability_threshold_low) {
        // directly invalid because traversability too low
        return LOW;
    }
    // need to check collision
    return MEDIUM;
}

bool StateValidityChecker::isValid(const ob::ScopedState<> state) { return isValid(state.get()->as<ob::State>()); }
