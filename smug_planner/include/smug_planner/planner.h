// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#pragma once

#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <smug_planner/planner_status.h>
#include <smug_planner/tsp_solver.h>
#include <smug_planner/validity_checker/validity_checker.h>
#include <smug_planner/validity_checker/validity_checker_validate_toi.h>

#include <mutex>

#include "smug_planner/map/map.h"
#include "smug_planner/motion_validator/inclination_motion_validator.h"
#include "smug_planner/motion_validator/inclination_motion_validator_validate_toi.h"
#include "smug_planner/planners/lazy_prm_star_mg.h"
#include "smug_planner/sampler.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace smug_planner {
ob::OptimizationObjectivePtr getObjective(const ob::SpaceInformationPtr& si, const ParamsConstPtr& params);

class Planner {
   public:
    using StateSpace = ob::SE3StateSpace;
    using StateType = typename StateSpace::StateType;

   protected:
    // Parameters.
    ParamsConstPtr params_;

    /* data */
    std::shared_ptr<og::SimpleSetup> ss_;
    std::shared_ptr<StateSpace> space_;

    std::shared_ptr<LazyPRMStarMG> planner_;

    InformedSamplerAllocator sampler_allocator_;

    std::shared_ptr<StateValidityChecker> checker_;
    std::shared_ptr<StateValidityCheckerValidateToI> checker_validate_toi_;

    std::shared_ptr<ob::DiscreteMotionValidator> discrete_motion_validator_;
    std::shared_ptr<InclinationMotionValidator> inclination_motion_validator_;
    std::shared_ptr<InclinationMotionValidatorValidateToI> inclination_motion_validator_validate_toi_;

    std::shared_ptr<Map> map_;
    mutable std::mutex map_mutex_;

    bool stop_planning_{false};

    double** path_lengths_;
    // double** center_path_lengths_;
    og::PathGeometricPtr complete_path_;
    og::PathGeometricPtr complete_path_dp_;
    og::PathGeometricPtr complete_path_idp_;
    og::PathGeometricPtr complete_path_rba_;

    og::PathGeometricPtr center_complete_path_;

    std::vector<std::vector<og::PathGeometric>> idp_paths_before_plan_;
    std::vector<std::vector<bool>> is_planned_;

    std::vector<og::PathGeometric> idp_paths_after_plan_;

    std::vector<int> center_sequence_;

    std::vector<og::PathGeometricPtr> path_buffer_ompl_;

    std::vector<int> center_idx_list_;  // used to find the index of the goal in goal vector, need to be cleaned after
                                        // or before every service call

   public:
    Planner(const ParamsConstPtr& params = std::make_shared<const Params>());
    ~Planner();
    void clear();

    PlannerStatus multiGoalPlan(const ob::ScopedState<>& start, const std::vector<ob::ScopedState<>>& goals);

    PlannerStatus multiGoalSetPlan(const ob::ScopedState<>& start,
                                   const std::vector<std::vector<ob::ScopedState<>>>& goal_sets);

    PlannerStatus dynamicProgramming(const std::vector<std::vector<ob::ScopedState<>>>& goal_sets,
                                     std::vector<ob::ScopedState<>> points);

    PlannerStatus iterativeDP(const std::vector<std::vector<ob::ScopedState<>>>& goal_sets,
                              std::vector<ob::ScopedState<>> points);

    PlannerStatus rubberBandAlgorithm(const std::vector<std::vector<ob::ScopedState<>>>& goal_sets,
                                      std::vector<ob::ScopedState<>> points,
                                      std::vector<ob::ScopedState<>> goal_centers);

    PlannerStatus singleGoalPlan(const ob::ScopedState<>& start, const ob::ScopedState<>& goal,
                                 bool using_all_time = false, double time = 0.1);

    og::PathGeometric getSingleSolutionPath(const bool& simplyfy = false) const;

    std::vector<std::vector<ob::ScopedState<>>> generateSmallCraterTargetSets(int n_set, int n_goal_per_set);

    ob::ScopedState<> generateSmallCraterStart();

    ob::ScopedState<> generateRandomPose();

    void setBounds();

    void setMapTsdfMap(TsdfMapPtr&& map);

    void setMapTraversabilityMap(TraversabilityMapPtr&& map);

    void setMapHeightMap(HeightMapPtr&& map);

    std::vector<ob::ScopedState<>> makeGoalVector(const std::vector<std::vector<ob::ScopedState<>>> goal_sets);

    int getIdxInVector(const int center_idx, const int pose_idx);

    void postProcessPath();
};

}  // namespace smug_planner