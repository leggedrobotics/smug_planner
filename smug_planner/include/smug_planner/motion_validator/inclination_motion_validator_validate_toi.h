// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#pragma once

#include <smug_planner/validity_checker/validity_checker_validate_toi.h>  // for the definition of ToI struct

#include "smug_planner/motion_validator/inclination_motion_validator.h"
#include "smug_planner/validity_type.h"

namespace ob = ompl::base;

namespace smug_planner {

class InclinationMotionValidatorValidateToI : public InclinationMotionValidator {
   public:
    InclinationMotionValidatorValidateToI(ob::SpaceInformation *si, const std::shared_ptr<Map> &map,
                                          const ParamsConstPtr &params)
        : InclinationMotionValidator(si, map, params) {}

    InclinationMotionValidatorValidateToI(const ob::SpaceInformationPtr &si, const std::shared_ptr<Map> &map,
                                          const ParamsConstPtr &params)
        : InclinationMotionValidator(si, map, params) {}

    bool withinToI(const ob::State *state) const;
    void addToI(const ob::ScopedState<> centroid, double radius);

    bool isValid(const ob::State *s1, const ob::State *s2) const override;
    bool isValid(const ob::State *s1, const ob::State *s2, const ob::State *s3) const override;

    ValidityType validityType(const ob::State *s1, const ob::State *s2) const;

   protected:
    std::vector<ToI> tois_ = {};
    double getDist(const ob::ScopedState<> p1, const ob::State *p2) const;
};

}  // namespace smug_planner