// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#include "smug_planner/objectives/path_length_objective.h"

using namespace smug_planner;

inline double getAngleDiff(double x, double y) {
    const double d = std::fabs(y - x);
    return (d > M_PI) ? 2.0 * M_PI - d : d;
}

PathLengthObjective::PathLengthObjective(const ob::SpaceInformationPtr &si, const ParamsConstPtr &params)
    : Base::PathLengthOptimizationObjective(si), params_(params), checker_(si->getStateValidityChecker()) {}
