// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#pragma once

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include "smug_planner/params.h"
#include "smug_planner/utils.h"
#include "smug_planner/validity_checker/validity_checker.h"

namespace ob = ompl::base;

namespace smug_planner {

class PathLengthObjective : public ob::PathLengthOptimizationObjective {
    // Parameters.
    ParamsConstPtr params_;

   protected:
    using Base = ob::PathLengthOptimizationObjective;

    ob::StateValidityCheckerPtr checker_;

   public:
    PathLengthObjective(const ob::SpaceInformationPtr& si, const ParamsConstPtr& params);
};

}  // namespace smug_planner
