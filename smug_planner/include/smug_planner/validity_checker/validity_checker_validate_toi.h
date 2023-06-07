// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#pragma once

#include <smug_planner/validity_checker/validity_checker.h>

namespace smug_planner {

struct ToI {
    ob::ScopedState<> centroid_;
    double radius_;
};

class StateValidityCheckerValidateToI : public StateValidityChecker {
   protected:
    std::vector<ToI> tois_ = {};

   public:
    virtual bool isValid(const ob::State *state) const override;

    ValidityType validityType(const ob::State *state) const override;

    bool withinToI(const ob::State *state) const;

    void addToI(const ob::ScopedState<> centroid, double radius);

    double getDist(const ob::ScopedState<> p1, const ob::ScopedState<> p2) const;

    double getDist(const ob::ScopedState<> p1, const ob::State *p2) const;

    StateValidityCheckerValidateToI(const ob::SpaceInformationPtr &si, const ParamsConstPtr &params);
};

}  // namespace smug_planner