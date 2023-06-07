// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#include "smug_planner/sampler.h"

#include <iostream>

#include "smug_planner/utils.h"

using namespace smug_planner;

inline double getAngleDiff(double x, double y) {
    const double d = std::fabs(y - x);
    return (d > M_PI) ? 2.0 * M_PI - d : d;
}

InformedSampler::InformedSampler(const ob::StateSpace* space, const std::shared_ptr<Map>& map,
                                 const ParamsConstPtr& params)
    : ob::StateSampler(space), params_(params), map_(map) {
    auto bounds = space_->as<ob::SE3StateSpace>()->getBounds();
    x_min_ = bounds.low[0];
    y_min_ = bounds.low[1];
    x_max_ = bounds.high[0];
    y_max_ = bounds.high[1];

    // set default SE2 sampler
    space_se2_ = std::make_shared<ob::SE2StateSpace>();
    ob::RealVectorBounds bounds_se2(2);
    bounds_se2.setLow(0, x_min_);
    bounds_se2.setHigh(0, x_max_);
    bounds_se2.setLow(1, y_min_);
    bounds_se2.setHigh(1, y_max_);
    space_se2_->setBounds(bounds_se2);
    default_sampler_ = space_se2_->allocDefaultStateSampler();
}

bool InformedSampler::sampleUniformInformed(ob::State* state, const ob::State* s1, const ob::State* s2,
                                            const double cost) {
    // only informed sample in x,y, then sample yaw randomly
    if (cost < double(INFINITY)) {
        // informed sampling
        auto s1_se3 = s1->as<ob::SE3StateSpace::StateType>();
        auto s2_se3 = s2->as<ob::SE3StateSpace::StateType>();
        // compute center of the ellipsoid
        double center_x = (s1_se3->getX() + s2_se3->getX()) / 2;
        double center_y = (s1_se3->getY() + s2_se3->getY()) / 2;

        auto s1_yaw = getYawFromSO3(s1_se3->rotation());
        auto s2_yaw = getYawFromSO3(s2_se3->rotation());
        auto yaw_diff = getAngleDiff(s1_yaw, s2_yaw);

        // get default weights of 3 components
        auto weights = space_->as<ob::CompoundStateSpace>()->getSubspaceWeights();

        // compute the cost from yaw mostion
        double cost_yaw = std::fabs(yaw_diff) * weights[1];
        // deduct the yaw cost from the total cost
        double cost_xy = (cost - cost_yaw) / weights[0];

        Eigen::Vector2d center(center_x, center_y);

        double main_axis_x = (s1_se3->getX() - s2_se3->getX()) / 2;
        double main_axis_y = (s1_se3->getY() - s2_se3->getY()) / 2;

        Eigen::Vector2d main_axis(main_axis_x, main_axis_y);
        auto first_axis = main_axis.normalized();
        auto second_axis = Eigen::Vector2d(first_axis[0], first_axis[1]);

        Eigen::Matrix2d Align;
        Align.block<2, 1>(0, 0) = first_axis;
        Align.block<2, 1>(0, 1) = second_axis;

        // compute radii of the ellipsoid
        double r_max = cost_xy / 2;
        double r_min_squared = r_max * r_max - main_axis.squaredNorm();
        if (r_min_squared <= 0) {
            // this could happen due to rounding errors
            // this means the cost is quite optimal
            return false;
        }
        double r_min = sqrt(r_min_squared);
        Eigen::DiagonalMatrix<double, 2> L(r_max, r_min);

        Eigen::Vector2d sample_final;
        do {
            // sample unit ball
            std::vector<double> sample{0.0, 0.0};
            rng_.uniformInBall(1, sample);
            Eigen::Vector2d unit_sample(sample[0], sample[1]);

            // transform to align with the main axis
            auto sample_aligned = Align * L * unit_sample;

            // add to center
            sample_final = Eigen::Vector2d(Eigen::Vector2d(sample_aligned) + center);

        } while (!inXYBounds(sample_final));

        // get height of the terrain
        float height;
        map_->getMapHeightDefaultInf(Eigen::Vector3f(sample_final[0], sample_final[1], 0), &height);
        auto state_se3 = state->as<ob::SE3StateSpace::StateType>();
        state_se3->setXYZ(sample_final[0], sample_final[1], height);

        // sample yaw randomly from -pi ~ pi
        // from yaw to quaternion
        setSO3FromYaw(state_se3->rotation(), M_PI * (2 * rng_.uniform01() - 1));
        return true;

    } else {
        sampleUniform(state);
        return true;
    }
}

void InformedSampler::sampleUniform(ob::State* state) {
    double x = (x_max_ - x_min_) * rng_.uniform01() + x_min_;
    double y = (y_max_ - y_min_) * rng_.uniform01() + y_min_;
    double yaw = M_PI * (2 * rng_.uniform01() - 1);
    auto state_se3 = state->as<ob::SE3StateSpace::StateType>();
    float height;
    map_->getMapHeightDefaultInf(Eigen::Vector3f(x, y, 0), &height);
    state_se3->setXYZ(x, y, height);
    // from yaw to quaternion
    setSO3FromYaw(state_se3->rotation(), yaw);
}

bool InformedSampler::inXYBounds(Eigen::Vector3d state) {
    bool valid = state[0] >= x_min_ && state[0] <= x_max_ && state[1] >= y_min_ && state[1] <= y_max_;
    return valid;
}

bool InformedSampler::inXYBounds(Eigen::Vector2d state) {
    bool valid = state[0] >= x_min_ && state[0] <= x_max_ && state[1] >= y_min_ && state[1] <= y_max_;
    return valid;
}

void InformedSamplerAllocator::setMap(const std::shared_ptr<Map>& map) { map_ = map; }

std::shared_ptr<InformedSampler> InformedSamplerAllocator::getSampler(const ob::StateSpace* space) {
    return std::make_shared<InformedSampler>(space, map_, params_);
}
