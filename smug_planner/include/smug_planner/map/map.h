// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#pragma once

#include <ompl/util/Time.h>
#include <voxblox/core/traversability_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/interpolator/interpolator.h>

#include <functional>
#include <memory>
#include <mutex>

#include "smug_planner/params.h"

namespace smug_planner {

using TsdfMapPtr = std::unique_ptr<voxblox::TsdfMap>;
using TraversabilityMapPtr = std::unique_ptr<voxblox::TraversabilityMap>;
using HeightMapPtr = std::unique_ptr<voxblox::Layer<voxblox::HeightVoxel>>;
using Interpolator = voxblox::Interpolator<voxblox::TsdfVoxel>;
using InterpolatorPtr = std::unique_ptr<Interpolator>;

class Map {
    TsdfMapPtr tsdf_map_ = nullptr;
    TraversabilityMapPtr traversability_map_ = nullptr;
    HeightMapPtr height_map_ = nullptr;

    // interpolator to get distance from voxblox map
    InterpolatorPtr tsdf_interpolator_ptr_ = nullptr;

    ParamsConstPtr params_;

    mutable std::mutex tsdf_mutex_;

    mutable std::mutex height_mutex_;

    mutable std::mutex traversability_mutex_;

   public:
    // Constructs an empty map, without underlying grid map or Voxblox map.
    Map(const ParamsConstPtr& params);

    // Sets the Voxblox map
    void setTsdfMap(TsdfMapPtr&& tsdf_map);

    void setTraversabilityMap(TraversabilityMapPtr&& traversability_map);

    void setHeightMap(HeightMapPtr&& height_map);

    inline bool hasTsdfMapSet() const {
        std::lock_guard<std::mutex> lock(tsdf_mutex_);
        return static_cast<bool>(tsdf_map_);
    }

    inline bool hasTraversabilityMapSet() const {
        std::lock_guard<std::mutex> lock(traversability_mutex_);
        return static_cast<bool>(traversability_map_);
    }

    inline bool hasHeightMapSet() const {
        std::lock_guard<std::mutex> lock(height_mutex_);
        return static_cast<bool>(height_map_);
    }

    inline bool hasMapSet() const { return hasTsdfMapSet() && hasTraversabilityMapSet() && hasHeightMapSet(); }

    // Copies the underlying map and parameters of another map object.
    void copy(const Map& map);

    inline const voxblox::TsdfMap& getTsdfMap() const {
        std::lock_guard<std::mutex> lock(tsdf_mutex_);
        return *tsdf_map_;
    }

    inline const voxblox::TraversabilityMap& getTraversabilityMap() const {
        std::lock_guard<std::mutex> lock(traversability_mutex_);
        return *traversability_map_;
    }

    inline const voxblox::Layer<voxblox::HeightVoxel>& getHeightMap() const {
        std::lock_guard<std::mutex> lock(height_mutex_);
        return *height_map_;
    }

    // finds distance to the nearest obstacle
    bool getMapDistance3D(const Eigen::Vector3f& position, float* distance) const {
        std::lock_guard<std::mutex> lock(tsdf_mutex_);
        return tsdf_interpolator_ptr_->getDistance(position, distance, true);
    }

    // finds traversability value at given point if it exists
    bool getMapTraversability(const Eigen::Vector3f& position, float* traversability) const {
        std::lock_guard<std::mutex> lock(traversability_mutex_);
        auto voxel_ptr = traversability_map_->getTraversabilityLayer().getVoxelPtrByCoordinates(position);
        // returns false if no traversability information is available
        if (!voxel_ptr || voxel_ptr->n_values == 0) return false;
        *traversability = voxel_ptr->traversability;
        return true;
    }

    // get the terrain height
    bool getMapHeight(const Eigen::Vector3f& position, float* height) const {
        std::lock_guard<std::mutex> lock(height_mutex_);
        auto voxel_ptr = height_map_->getVoxelPtrByCoordinates(position);
        // returns false if no traversability information is available
        if (!voxel_ptr || voxel_ptr->n_values == 0) return false;
        *height = voxel_ptr->height;
        return true;
    }

    bool getMapHeightDefaultInf(const Eigen::Vector3f& position, float* height) const {
        bool get_height_success = getMapHeight(position, height);
        if (!get_height_success) {
            *height = INFINITY;
            return false;
        } else {
            return true;
        }
    }
};

}  // namespace smug_planner
