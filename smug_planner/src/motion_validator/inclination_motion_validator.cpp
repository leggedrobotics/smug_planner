// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#include "smug_planner/motion_validator/inclination_motion_validator.h"

#include <ompl/base/spaces/SE3StateSpace.h>

#include <iomanip>
#include <queue>

#include "ompl/util/Exception.h"

using namespace smug_planner;

void InclinationMotionValidator::defaultSettings() {
    stateSpace_ = si_->getStateSpace().get();
    if (stateSpace_ == nullptr) throw ompl::Exception("No state space for motion validator");
}

bool InclinationMotionValidator::checkMotion(const ob::State *s1, const ob::State *s2,
                                             std::pair<ob::State *, double> &lastValid) const {
    /* assume motion starts in a valid configuration so s1 is valid */
    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    ob::State *segment_start = si_->cloneState(s1);
    if (nd > 1) {
        /* temporary storage for the checked state */
        ob::State *segment_end = si_->allocState();

        for (int j = 1; j < nd; ++j) {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, segment_end);
            // get height for test;

            if (!isValid(segment_start, segment_end)) {
                lastValid.second = (double)(j - 1) / (double)nd;
                if (lastValid.first != nullptr) stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
                result = false;
                break;
            }
            segment_start = segment_end;
        }
        si_->freeState(segment_end);
    }

    if (result)  // the last state remains to check
        if (!isValid(segment_start, s2)) {
            lastValid.second = (double)(nd - 1) / (double)nd;
            if (lastValid.first != nullptr) stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
            result = false;
        }

    si_->freeState(segment_start);

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}

bool InclinationMotionValidator::checkMotion(const ob::State *s1, const ob::State *s2) const {
    /* assume motion starts in a valid configuration so s1 is valid */
    ob::State *segment_start = si_->cloneState(s1);
    if (!isValid(segment_start, s2))  // todo : modify here
    {
        invalid_++;
        return false;
    }

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

            pos.pop();

            if (x.first < mid) pos.emplace(x.first, mid - 1);
            if (x.second > mid) pos.emplace(mid + 1, x.second);
        }

        si_->freeState(first_state);
        si_->freeState(mid_state);
        si_->freeState(second_state);
    }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}

bool InclinationMotionValidator::isValid(const ob::State *s1, const ob::State *s2) const {
    // get map height for s1 s2, make them immediately on the ground
    auto s1_se3 = si_->cloneState(s1)->as<ob::SE3StateSpace::StateType>();
    auto s2_se3 = si_->cloneState(s2)->as<ob::SE3StateSpace::StateType>();

    float s1_height;
    float s2_height;

    auto s1_x = s1_se3->getX();
    auto s1_y = s1_se3->getY();

    auto s2_x = s2_se3->getX();
    auto s2_y = s2_se3->getY();

    if (!map_->getMapHeight(Eigen::Vector3f(s1_x, s1_y, 0), &s1_height)) {
        return false;
    }

    if (!map_->getMapHeight(Eigen::Vector3f(s2_x, s2_y, 0), &s2_height)) {
        return false;
    }

    // compute inclination
    double inclination =
        abs(s1_height - s2_height) / sqrt((s1_x - s2_x) * (s1_x - s2_x) + (s1_y - s2_y) * (s1_y - s2_y));

    if (inclination > params_->motion_validator.max_inclination) {
        return false;
    }

    // only check collision for s2
    s2_se3->setZ(s2_height);

    return si_->isValid(s2_se3);
}

bool InclinationMotionValidator::isValid(const ob::State *s1, const ob::State *s2, const ob::State *s3) const {
    // get map height for s1 s2, make them immediately on the ground
    auto s1_se3 = si_->cloneState(s1)->as<ob::SE3StateSpace::StateType>();
    auto s2_se3 = si_->cloneState(s2)->as<ob::SE3StateSpace::StateType>();
    auto s3_se3 = si_->cloneState(s2)->as<ob::SE3StateSpace::StateType>();

    float s1_height;
    float s2_height;
    float s3_height;

    auto s1_x = s1_se3->getX();
    auto s1_y = s1_se3->getY();

    auto s2_x = s2_se3->getX();
    auto s2_y = s2_se3->getY();

    auto s3_x = s3_se3->getX();
    auto s3_y = s3_se3->getY();

    if (!map_->getMapHeight(Eigen::Vector3f(s1_x, s1_y, 0), &s1_height)) {
        return false;
    }

    if (!map_->getMapHeight(Eigen::Vector3f(s2_x, s2_y, 0), &s2_height)) {
        return false;
    }

    if (!map_->getMapHeight(Eigen::Vector3f(s3_x, s3_y, 0), &s3_height)) {
        return false;
    }

    // compute inclination
    double inclination_1 =
        abs(s1_height - s2_height) / sqrt((s1_x - s2_x) * (s1_x - s2_x) + (s1_y - s2_y) * (s1_y - s2_y));
    if (inclination_1 > params_->motion_validator.max_inclination) {
        return false;
    }

    double inclination_2 =
        abs(s3_height - s2_height) / sqrt((s3_x - s2_x) * (s3_x - s2_x) + (s3_y - s2_y) * (s3_y - s2_y));
    if (inclination_2 > params_->motion_validator.max_inclination) {
        return false;
    }

    // only check collision for s2
    s2_se3->setZ(s2_height);
    return si_->isValid(s2_se3);
}
