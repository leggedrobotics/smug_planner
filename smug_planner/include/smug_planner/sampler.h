// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#pragma once

#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>

#include "smug_planner/map/map.h"
#include "smug_planner/params.h"

namespace ob = ompl::base;

namespace smug_planner {

class InformedSampler : public ob::StateSampler {
    // Parameters.
    ParamsConstPtr params_;

    ob::StateSamplerPtr default_sampler_;
    std::shared_ptr<ob::SE2StateSpace> space_se2_;

    double x_min_;
    double x_max_;
    double y_min_;
    double y_max_;

    std::shared_ptr<Map> map_;

   public:
    InformedSampler(const ob::StateSpace* si, const std::shared_ptr<Map>& map, const ParamsConstPtr& params);

    virtual void sampleUniform(ob::State* state) override;

    bool sampleUniformInformed(ob::State* state, const ob::State* s1, const ob::State* s2, const double cost);

    virtual void sampleUniformNear(ob::State* state, const ob::State* near, double distance) override{};

    virtual void sampleGaussian(ob::State* state, const ob::State* mean, double std_dev) override{};

    bool inXYBounds(Eigen::Vector3d state);

    bool inXYBounds(Eigen::Vector2d state);
};

class InformedSamplerAllocator {
    // Parameters.
    ParamsConstPtr params_;

    std::shared_ptr<Map> map_;

   public:
    InformedSamplerAllocator(const ParamsConstPtr& params) : params_(params) {}

    void setMap(const std::shared_ptr<Map>& map);

    std::shared_ptr<InformedSampler> getSampler(const ob::StateSpace* space);
};

}  // namespace smug_planner
