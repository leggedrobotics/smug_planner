// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#pragma once

#include <ompl/base/spaces/SE3StateSpace.h>

#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace ob = ompl::base;

namespace smug_planner {

using Scalar = float;

using Pose3 = Eigen::Transform<Scalar, 3, Eigen::Affine>;

inline Pose3 Pose3FromSE3(const ob::State* state) {
    const auto state_se3 = state->as<ob::SE3StateSpace::StateType>();
    const auto& rotation = state_se3->rotation();
    Pose3 pose;
    pose.translation().x() = state_se3->getX();
    pose.translation().y() = state_se3->getY();
    pose.translation().z() = state_se3->getZ();
    pose.matrix().topLeftCorner(3, 3) =
        Eigen::Quaternion<Scalar>(rotation.w, rotation.x, rotation.y, rotation.z).toRotationMatrix();
    return pose;
}

inline Pose3 Pose3FromXYZ(Scalar x, Scalar y, Scalar z) {
    Pose3 pose = Pose3::Identity();
    pose.translation().x() = x;
    pose.translation().y() = y;
    pose.translation().z() = z;
    return pose;
}

template <typename T>
inline T getYawFromQuat(T w, T x, T y, T z) {
    return atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

inline Scalar getYawFromSO3(const ob::SO3StateSpace::StateType& s) { return getYawFromQuat(s.w, s.x, s.y, s.z); }

inline void setSO3FromYaw(ob::SO3StateSpace::StateType& s, double yaw) {
    s.w = cos(yaw / 2);
    s.x = 0;
    s.y = 0;
    s.z = sin(yaw / 2);
}

}  // namespace smug_planner
