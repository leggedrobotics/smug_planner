// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#include "smug_planner/motion_validator/inclination_motion_validator_validate_toi.h"

#include <ompl/base/spaces/SE3StateSpace.h>

#include <iomanip>
#include <queue>

#include "ompl/util/Exception.h"

using namespace smug_planner;

ValidityType InclinationMotionValidatorValidateToI::validityType(const ob::State *s1, const ob::State *s2) const {
    /* assume motion starts in a valid configuration so s1 is valid */
    bool within_toi = false;
    ob::State *segment_start = si_->cloneState(s1);
    if (!isValid(segment_start, s2)) {
        invalid_++;
        return ValidityType::COLLISION;
    }

    within_toi = within_toi || (withinToI(segment_start) || withinToI(s2));

    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    /* initialize the queue of test positions */
    std::queue<std::pair<int, int>> pos;
    if (nd >= 2) {
        pos.emplace(1, nd - 1);

        /* temporary storage for the checked state */
        ob::State *first_state = si_->allocState();
        ob::State *mid_state = si_->allocState();
        ob::State *second_state = si_->allocState();

        /* repeatedly subdivide the path segment in the middle (and check the middle) */
        while (!pos.empty()) {
            std::pair<int, int> x = pos.front();

            int mid = (x.first + x.second) / 2;

            stateSpace_->interpolate(s1, s2, (double)x.first / (double)nd, first_state);
            stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, mid_state);
            stateSpace_->interpolate(s1, s2, (double)x.second / (double)nd, second_state);

            if (!isValid(first_state, mid_state, second_state)) {
                result = false;
                break;
            }
            within_toi = within_toi || (withinToI(first_state) || withinToI(mid_state) || withinToI(second_state));

            pos.pop();

            if (x.first < mid) pos.emplace(x.first, mid - 1);
            if (x.second > mid) pos.emplace(mid + 1, x.second);
        }

        si_->freeState(first_state);
        si_->freeState(mid_state);
        si_->freeState(second_state);
    }

    if (result) {
        valid_++;
        if (within_toi) {
            return ValidityType::WITHIN_TOI;
        }
        return ValidityType::VALID;
    } else {
        invalid_++;
        return ValidityType::COLLISION;
    }
}

bool InclinationMotionValidatorValidateToI::isValid(const ob::State *s1, const ob::State *s2) const {
    if (withinToI(s1) || withinToI(s2)) {
        return true;
    }
    return InclinationMotionValidator::isValid(s1, s2);
}

bool InclinationMotionValidatorValidateToI::isValid(const ob::State *s1, const ob::State *s2,
                                                    const ob::State *s3) const {
    if (withinToI(s2)) {
        return true;
    }
    // case of s2 outside toi
    bool s1_within_toi = withinToI(s1);
    bool s3_within_toi = withinToI(s3);
    if (s1_within_toi) {
        if (s3_within_toi) {
            return true;
        } else {
            return InclinationMotionValidator::isValid(s2, s3);
        }
    }
    // case of s1, s2 outside toi
    if (s3_within_toi) {
        return InclinationMotionValidator::isValid(s1, s2);
    } else {
        return InclinationMotionValidator::isValid(s1, s2, s3);
    }
}

bool InclinationMotionValidatorValidateToI::withinToI(const ob::State *state) const {
    double l = params_->robot.torso.length;
    double w = params_->robot.torso.width;
    double h = params_->robot.torso.height;
    double r = sqrt(l * l + w * w + h * h) / 2;

    for (ToI toi : tois_) {
        if (getDist(toi.centroid_, state) <= (toi.radius_ * 1.1 + r)) {
            return true;
        }
    }
    return false;
}

void InclinationMotionValidatorValidateToI::addToI(const ob::ScopedState<> centroid, double r) {
    ToI toi{centroid, r};
    tois_.push_back(toi);
}

double InclinationMotionValidatorValidateToI::getDist(const ob::ScopedState<> p1, const ob::State *p2) const {
    auto p1_se2 = p1.get()->as<ob::SE3StateSpace::StateType>();
    auto p2_se2 = p2->as<ob::SE3StateSpace::StateType>();
    auto dist_x = p1_se2->getX() - p2_se2->getX();
    auto dist_y = p1_se2->getY() - p2_se2->getY();
    auto dist_z = p1_se2->getZ() - p2_se2->getZ();
    return sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);
}