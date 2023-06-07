// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#pragma once

#include <ompl/base/ScopedState.h>

#include "ompl/base/MotionValidator.h"
#include "ompl/base/SpaceInformation.h"
#include "smug_planner/map/map.h"
#include "smug_planner/params.h"

namespace ob = ompl::base;

namespace smug_planner {

class InclinationMotionValidator : public ob::MotionValidator {
   public:
    InclinationMotionValidator(ob::SpaceInformation *si, const std::shared_ptr<Map> &map, const ParamsConstPtr &params)
        : ob::MotionValidator(si), params_(params), map_(map) {
        defaultSettings();
    }

    InclinationMotionValidator(const ob::SpaceInformationPtr &si, const std::shared_ptr<Map> &map,
                               const ParamsConstPtr &params)
        : ob::MotionValidator(si), params_(params), map_(map) {
        defaultSettings();
    }

    ~InclinationMotionValidator() override = default;

    bool checkMotion(const ob::State *s1, const ob::State *s2) const override;

    bool checkMotion(const ob::State *s1, const ob::State *s2,
                     std::pair<ob::State *, double> &lastValid) const override;

    virtual bool isValid(const ob::State *s1, const ob::State *s2) const;

    virtual bool isValid(const ob::State *s1, const ob::State *s2, const ob::State *s3) const;

   protected:
    ob::StateSpace *stateSpace_;
    void defaultSettings();
    ParamsConstPtr params_;
    std::shared_ptr<Map> map_;
};

}  // namespace smug_planner