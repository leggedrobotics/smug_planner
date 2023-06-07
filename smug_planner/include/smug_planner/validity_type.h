// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#pragma once

namespace smug_planner {

enum ValidityType {
    UNCHECKED = 0,
    VALID,
    NO_HEIGHT,
    COLLISION,
    TRAVERSABILITY_LOW,
    TRAVERSABILITY_HIGH,
    WITHIN_TOI,
};

}
