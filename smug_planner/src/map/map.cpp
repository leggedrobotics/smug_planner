// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#include "smug_planner/map/map.h"

#include <random>

using namespace smug_planner;

void Map::setTsdfMap(TsdfMapPtr&& tsdf_map_new) {
    std::lock_guard<std::mutex> lock(tsdf_mutex_);
    const TsdfMapPtr tsdf_map_old = std::move(tsdf_map_);
    tsdf_map_ = std::move(tsdf_map_new);

    // sets the interpolator to the current map
    tsdf_interpolator_ptr_ = std::make_unique<Interpolator>(tsdf_map_->getTsdfLayerPtr());
}

void Map::setTraversabilityMap(TraversabilityMapPtr&& traversability_map_new) {
    std::lock_guard<std::mutex> lock(traversability_mutex_);
    const TraversabilityMapPtr traversability_map_old = std::move(traversability_map_);
    traversability_map_ = std::move(traversability_map_new);
}

void Map::setHeightMap(HeightMapPtr&& height_map_new) {
    std::lock_guard<std::mutex> lock(height_mutex_);
    const HeightMapPtr height_map_old = std::move(height_map_);
    height_map_ = std::move(height_map_new);
}

void Map::copy(const Map& map) {
    std::lock_guard<std::mutex> tsdf_lock(tsdf_mutex_);
    std::lock_guard<std::mutex> trav_lock(traversability_mutex_);
    std::lock_guard<std::mutex> height_lock(height_mutex_);

    *tsdf_map_ = *map.tsdf_map_;
    *traversability_map_ = *map.traversability_map_;
    *tsdf_interpolator_ptr_ = *map.tsdf_interpolator_ptr_;
    *height_map_ = *map.height_map_;

    params_ = map.params_;
}

Map::Map(const ParamsConstPtr& params) : params_(params) {}
