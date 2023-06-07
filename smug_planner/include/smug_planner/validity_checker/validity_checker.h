// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#pragma once
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include "smug_planner/map/map.h"
#include "smug_planner/validity_type.h"
#include "validity_checker_body.h"

namespace ob = ompl::base;

namespace smug_planner {

enum TraversabilityLevel { LOW = 0, MEDIUM = 1, HIGH = 2 };

class StateValidityChecker : public ob::StateValidityChecker {
   protected:
    ValidityCheckerBody checker_body_;

    std::shared_ptr<Map> map_;
    ParamsConstPtr params_;

   public:
    StateValidityChecker(const ob::SpaceInformationPtr &si, const ParamsConstPtr &params);

    void setMap(const std::shared_ptr<Map> &map);

    bool hasMap() const;

    virtual bool isValid(const ob::State *state) const override;

    bool isValid(const ob::ScopedState<> state);  // for checking validity while generating ToIs for testing

    virtual ValidityType validityType(const ob::State *state) const;

    bool bodyNoCollision(const Pose3 &pose) const;

    ValidityType validityType(const ob::State *s1, const ob::State *s2,
                              std::pair<ob::State *, double> &lastValid) const;

    ValidityType validityType(const ob::State *s1, const ob::State *s2) const;

    TraversabilityLevel getTraversabilityLevel(const Pose3 &pose) const;
};

}  // namespace smug_planner
