// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#include "smug_planner/validity_checker/validity_checker_validate_toi.h"

using namespace smug_planner;

StateValidityCheckerValidateToI::StateValidityCheckerValidateToI(const ob::SpaceInformationPtr &si,
                                                                 const ParamsConstPtr &params)
    : StateValidityChecker(si, params) {}

bool StateValidityCheckerValidateToI::isValid(const ob::State *state) const {
    if (withinToI(state)) {
        return true;
    }
    return StateValidityChecker::isValid(state);
}

ValidityType StateValidityCheckerValidateToI::validityType(const ob::State *state) const {
    if (withinToI(state)) {
        return ValidityType::WITHIN_TOI;
    }
    return StateValidityChecker::validityType(state);
}

void StateValidityCheckerValidateToI::addToI(const ob::ScopedState<> centroid, double r) {
    ToI toi{centroid, r};
    tois_.push_back(toi);
}

bool StateValidityCheckerValidateToI::withinToI(const ob::State *state) const {
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

double StateValidityCheckerValidateToI::getDist(const ob::ScopedState<> p1, const ob::ScopedState<> p2) const {
    auto p1_se2 = p1.get()->as<ob::SE3StateSpace::StateType>();
    auto p2_se2 = p2.get()->as<ob::SE3StateSpace::StateType>();
    auto dist_x = p1_se2->getX() - p2_se2->getX();
    auto dist_y = p1_se2->getY() - p2_se2->getY();
    auto dist_z = p1_se2->getZ() - p2_se2->getZ();
    return sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);
}

double StateValidityCheckerValidateToI::getDist(const ob::ScopedState<> p1, const ob::State *p2) const {
    auto p1_se2 = p1.get()->as<ob::SE3StateSpace::StateType>();
    auto p2_se2 = p2->as<ob::SE3StateSpace::StateType>();
    auto dist_x = p1_se2->getX() - p2_se2->getX();
    auto dist_y = p1_se2->getY() - p2_se2->getY();
    auto dist_z = p1_se2->getZ() - p2_se2->getZ();
    return sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);
}