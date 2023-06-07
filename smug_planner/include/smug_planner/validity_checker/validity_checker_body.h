// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#pragma once

#include <memory>
#include <mutex>

#include "smug_planner/map/map.h"
#include "smug_planner/params.h"
#include "smug_planner/utils.h"
#include "voxblox_box_checker.h"

namespace smug_planner {

class ValidityCheckerBody {
    using MapPtr = std::shared_ptr<Map>;

    ParamsConstPtr params_;

    MapPtr map_;
    std::unique_ptr<VoxbloxBoxChecker> checker3d_;
    mutable std::mutex mutex_;

   public:
    explicit ValidityCheckerBody(const ParamsConstPtr& params);

    void setMap(const MapPtr& map);

    bool isValid(const Pose3& pose) const;

    bool hasMap() const;
};

}  // namespace smug_planner
