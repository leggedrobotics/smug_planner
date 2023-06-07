// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#pragma once

namespace smug_planner {

enum PlannerStatus { UNKNOWN = 0, INVALID_START, INVALID_GOAL, NO_MAP, NOT_SOLVED, SOLVED, PREEMPTED };

}
