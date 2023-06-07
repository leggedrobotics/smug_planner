// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#include "smug_planner/planner.h"

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/Console.h>

#include <map>

#include "smug_planner/objectives/path_length_objective.h"

using namespace smug_planner;

Planner::Planner(const ParamsConstPtr& params)
    : params_(params), map_(std::make_shared<Map>(params)), sampler_allocator_(params_) {
    OMPL_INFORM("Instansiating Planner object ... ...");
    space_ = std::make_shared<StateSpace>();
    ss_ = std::make_shared<og::SimpleSetup>(space_);
    complete_path_ = std::make_shared<og::PathGeometric>(ss_->getSpaceInformation());
    complete_path_dp_ = std::make_shared<og::PathGeometric>(ss_->getSpaceInformation());
    complete_path_idp_ = std::make_shared<og::PathGeometric>(ss_->getSpaceInformation());
    complete_path_rba_ = std::make_shared<og::PathGeometric>(ss_->getSpaceInformation());
    center_complete_path_ = std::make_shared<og::PathGeometric>(ss_->getSpaceInformation());
    stop_planning_ = false;
    // set bounds for space_
    setBounds();

    auto si = ss_->getSpaceInformation();
    // ob::PlannerPtr planner;
    // planner.reset(new og::LazyPRMstar(si));
    // planner.reset(new LazyPRMStarMG(si, params_));

    planner_ = std::make_shared<LazyPRMStarMG>(si, params_);

    ss_->setPlanner(planner_);
    planner_->setProblemDefinition(ss_->getProblemDefinition());

    // Set sampler
    sampler_allocator_.setMap(map_);
    space_->setStateSamplerAllocator(
        std::bind(&InformedSamplerAllocator::getSampler, sampler_allocator_, std::placeholders::_1));

    checker_ = std::make_shared<StateValidityChecker>(si, params_);
    checker_->setMap(map_);
    checker_validate_toi_ = std::make_shared<StateValidityCheckerValidateToI>(si, params_);
    checker_validate_toi_->setMap(map_);
    ss_->setStateValidityChecker(checker_);

    inclination_motion_validator_ = std::make_shared<InclinationMotionValidator>(si, map_, params_);
    inclination_motion_validator_validate_toi_ =
        std::make_shared<InclinationMotionValidatorValidateToI>(si, map_, params_);
    discrete_motion_validator_ = std::make_shared<ob::DiscreteMotionValidator>(si);
    si->setMotionValidator(inclination_motion_validator_);

    double max_ext = ss_->getStateSpace()->getMaximumExtent();
    ss_->getStateSpace()->setLongestValidSegmentFraction(params_->planner.longest_valid_segment_length / max_ext);

    // Set objective.
    ss_->setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si)));

    //
    OMPL_INFORM("Instansiating Planner object completed");
}

ob::OptimizationObjectivePtr smug_planner::getObjective(const ob::SpaceInformationPtr& si,
                                                        const ParamsConstPtr& params) {
    auto opt = std::make_shared<ob::MultiOptimizationObjective>(si);

    ob::OptimizationObjectivePtr length_obj(new PathLengthObjective(si, params));
    opt->addObjective(length_obj, 1.0);

    return std::dynamic_pointer_cast<ob::OptimizationObjective>(opt);
}

void Planner::setMapTsdfMap(TsdfMapPtr&& map) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_->setTsdfMap(std::move(map));
}

void Planner::setMapTraversabilityMap(TraversabilityMapPtr&& map) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_->setTraversabilityMap(std::move(map));
}

void Planner::setMapHeightMap(HeightMapPtr&& map) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_->setHeightMap(std::move(map));
}

Planner::~Planner() {}

void Planner::clear() {
    ss_->clear();
    complete_path_->clear();
    complete_path_dp_->clear();
    complete_path_idp_->clear();
    complete_path_rba_->clear();
    center_complete_path_->clear();
    path_buffer_ompl_ = {};
    idp_paths_before_plan_ = {};
    is_planned_ = {};
    idp_paths_after_plan_ = {};
}

PlannerStatus Planner::multiGoalPlan(const ob::ScopedState<>& start, const std::vector<ob::ScopedState<>>& goals) {
    // the planned visiting sequence is stored in class member center_sequence_

    OMPL_INFORM("multiGoalPlan : Start multi-goal planning ... ...");
    clear();
    // compute target to target path;
    std::vector<ob::ScopedState<>> points = goals;
    // 0 ~ (n-1) th points are goals the n th point is the start
    points.push_back(start);
    int n_points = points.size();  // n_points is the number of goals plus start

    og::PathGeometricPtr** path_matrix;
    std::vector<std::vector<double>> center_path_lengths(n_points, std::vector<double>(n_points, 0));

    path_matrix = new og::PathGeometricPtr*[n_points];
    for (int i = 0; i < n_points; i++) {
        path_matrix[i] = new og::PathGeometricPtr[n_points];
    }
    double time;
    for (int i = 0; i < n_points - 1; i++) {
        for (int j = i + 1; j < n_points; j++) {
            if (stop_planning_) {
                return PREEMPTED;
            }
            ob::ScopedState<StateSpace> start(points[i]);
            ob::ScopedState<StateSpace> end(points[j]);

            PlannerStatus solved = singleGoalPlan(points[i], points[j]);
            // switch (solved)
            // {
            //     case PlannerStatus::UNKNOWN: return PlannerStatus::UNKNOWN;
            //     case PlannerStatus::NOT_SOLVED: return PlannerStatus::NOT_SOLVED;
            // }
            if (solved != SOLVED) {
                return NOT_SOLVED;
            }

            auto path = getSingleSolutionPath();

            // og::PathGeometric path = og::PathGeometric(ss_->getSpaceInformation(), start.get(), end.get());

            std::shared_ptr<og::PathGeometric> path_ptr;
            path_ptr = std::make_shared<og::PathGeometric>(path);

            // // reversed path
            og::PathGeometric path_reverse(path);
            path_reverse.reverse();

            std::shared_ptr<og::PathGeometric> path_reverse_ptr;
            path_reverse_ptr = std::make_shared<og::PathGeometric>(path_reverse);

            path_matrix[i][j] = path_ptr;
            path_matrix[j][i] = path_reverse_ptr;

            center_path_lengths[i][j] = path.length();
            center_path_lengths[j][i] = path.length();
            OMPL_INFORM("multiGoalPlan : Solution %d to %d completed", i, j);
        }
    }

    ompl::time::point start_tsp = ompl::time::now();
    tsp_solver::DataModel data_model;
    data_model.distance_matrix = center_path_lengths;
    data_model.depot = n_points - 1;

    TSP_Solver tsp_solver(data_model);
    center_sequence_ = tsp_solver.solve();  // center_sequence contains all goals including start

    OMPL_INFORM("Center sequence is : ");
    for (int i : center_sequence_) {
        OMPL_INFORM("%d", i);
    }
    if (stop_planning_) {
        return PREEMPTED;
    }

    OMPL_INFORM("multiGoalPlan : TSP solved");
    double tsp_time = ompl::time::seconds(ompl::time::now() - start_tsp);

    // center_sequence_ = {0:t1, 1:t2, 2:t3, ...}
    for (int n = 0; n < n_points; n++) {
        int cur_index = center_sequence_[n];
        int next_index;
        if (n == n_points - 1) {
            next_index = center_sequence_[0];
        } else {
            next_index = center_sequence_[n + 1];
        }
        og::PathGeometricPtr path_ptr_temp = path_matrix[cur_index][next_index];
        og::PathGeometric path_temp = *path_ptr_temp;
        center_complete_path_->append(path_temp);
        size_t n_pose = center_complete_path_->getStateCount();
    }

    OMPL_INFORM("Glue finished");

    // free memory
    for (int i = 0; i < n_points; i++) {
        delete[] path_matrix[i];
    }
    delete[] path_matrix;

    OMPL_INFORM("multiGoalPlan: Solved");
    return SOLVED;
}

PlannerStatus Planner::multiGoalSetPlan(const ob::ScopedState<>& start,
                                        const std::vector<std::vector<ob::ScopedState<>>>& goal_sets) {
    // used to be tuning helper, now just the one used to plan
    ompl::time::point start_time = ompl::time::now();
    OMPL_INFORM("multiGoalSetPlan : Start multi-goal set planning ... ...");
    int n_set = goal_sets.size();
    if (n_set == 0) {
        OMPL_INFORM("multiGoalSetPlan : Empty mission");
        return SOLVED;
    }
    OMPL_INFORM("multiGoalSetPlan : n_set = %d", n_set);

    clear();
    // first find the set sequence
    // solve tsp with the center of the goals
    std::vector<ob::ScopedState<>> goal_centers;
    for (auto goal_set : goal_sets) {
        goal_centers.push_back(goal_set[0]);
    }

    // here tois are not considered as obstacle, set checker temporarily to validate tois
    planner_->setStage(1);
    planner_->setSolveMethod(LazyPRMStarMG::SOLVE_MG_INFORMED);
    ss_->setStateValidityChecker(checker_validate_toi_);
    ss_->getSpaceInformation()->setMotionValidator(inclination_motion_validator_validate_toi_);

    PlannerStatus multi_goal_plan_solved = multiGoalPlan(start, goal_centers);
    if (multi_goal_plan_solved == PREEMPTED) {
        return PREEMPTED;
    }
    if (multi_goal_plan_solved != SOLVED) {
        OMPL_INFORM("multi goal plan not solved : aborting ... ...");
        return NOT_SOLVED;
    }

    double duration_til_2_stage = ompl::time::seconds(ompl::time::now() - start_time);

    // since here first stage is over
    // now tois are also obstacles, change validity checker
    planner_->setStage(2);
    ss_->setStateValidityChecker(checker_);
    ss_->getSpaceInformation()->setMotionValidator(inclination_motion_validator_);

    // convert 2d array of goal to a vector
    std::vector<ob::ScopedState<>> goals = makeGoalVector(goal_sets);

    // compute target to target path;
    std::vector<ob::ScopedState<>> points = goals;
    // 0 ~ (n-1) th points are goals the n th point is the start
    points.push_back(start);

    double duration_after_dp;
    double duration_after_idp;
    double duration_after_rba;

    if (params_->benchmark.dynamic_programming || params_->benchmark.compare) {
        PlannerStatus dp_status = dynamicProgramming(goal_sets, points);

        switch (dp_status) {
            case PlannerStatus::SOLVED:
                break;
            default:
                return NOT_SOLVED;
        }
    }
    duration_after_dp = ompl::time::seconds(ompl::time::now() - start_time);

    if (params_->benchmark.compare) {
        planner_->setSolveMethod(LazyPRMStarMG::SOLVE_MG_GET);
    }

    if (params_->benchmark.rubber_band_algorithm || params_->benchmark.compare) {
        PlannerStatus rba_status = rubberBandAlgorithm(goal_sets, points, goal_centers);

        switch (rba_status) {
            case PlannerStatus::SOLVED:
                break;
            default:
                return NOT_SOLVED;
        }
        OMPL_INFORM("multiGoalPlan: Solved");
    }
    duration_after_rba = ompl::time::seconds(ompl::time::now() - start_time);

    if (params_->benchmark.iterativeDP || params_->benchmark.compare) {
        PlannerStatus idp_status = iterativeDP(goal_sets, points);

        switch (idp_status) {
            case PlannerStatus::SOLVED:
                break;
            default:
                return NOT_SOLVED;
        }
    }
    duration_after_idp = ompl::time::seconds(ompl::time::now() - start_time);

    if (params_->benchmark.dynamic_programming || params_->benchmark.compare) {
        OMPL_INFORM("====== DP Path length : %f ======", complete_path_dp_->length());
        OMPL_INFORM("Finished in %f s", duration_after_dp);
        OMPL_INFORM("Break down to : first stage %f s, second stage (dp) %f s", duration_til_2_stage,
                    duration_after_dp - duration_til_2_stage);
    }

    if (params_->benchmark.rubber_band_algorithm || params_->benchmark.compare) {
        OMPL_INFORM("====== RBA Path length : %f ======", complete_path_rba_->length());
        OMPL_INFORM("Finished in %f s", duration_til_2_stage + duration_after_rba - duration_after_dp);
        OMPL_INFORM("Break down to : first stage %f s, second stage (rba) %f s", duration_til_2_stage,
                    duration_after_rba - duration_after_dp);
    }

    if (params_->benchmark.iterativeDP || params_->benchmark.compare) {
        OMPL_INFORM("====== iterative DP Path length : %f ======", complete_path_idp_->length());
        OMPL_INFORM("Finished in %f s", duration_til_2_stage + duration_after_idp - duration_after_rba);
        OMPL_INFORM("Break down to : first stage %f s, second stage (dp) %f s", duration_til_2_stage,
                    duration_after_idp - duration_after_rba);
    }

    // post process
    postProcessPath();

    return SOLVED;
}

PlannerStatus Planner::dynamicProgramming(const std::vector<std::vector<ob::ScopedState<>>>& goal_sets,
                                          std::vector<ob::ScopedState<>> points) {
    int n_points = points.size();
    int n_set = goal_sets.size();
    og::PathGeometricPtr** path_matrix;
    path_lengths_ = new double*[n_points];
    path_matrix = new og::PathGeometricPtr*[n_points];
    for (int i = 0; i < n_points; i++) {
        path_lengths_[i] = new double[n_points];
        path_matrix[i] = new og::PathGeometricPtr[n_points];
    }

    // dynamic programming and plannning during steps

    auto center_sequence_temp = center_sequence_;
    std::vector<double> cost_vector;
    // the last one goal to start

    int cur_center = center_sequence_temp.back();
    center_sequence_temp.pop_back();
    std::vector<std::vector<int>> next_point_table(n_set - 1);
    int next_point = n_points - 1;
    for (int i = 0; i < goal_sets[cur_center].size() - 1; i++) {
        if (stop_planning_) {
            return PREEMPTED;
        }
        int cur_point = getIdxInVector(cur_center, i);
        // plan p2p

        PlannerStatus solved = singleGoalPlan(points[cur_point], points[next_point]);
        og::PathGeometric path(ss_->getSpaceInformation());
        std::shared_ptr<og::PathGeometric> path_ptr;
        switch (solved) {
            case PlannerStatus::SOLVED:
                path = getSingleSolutionPath();
                path_ptr = std::make_shared<og::PathGeometric>(path);
                path_matrix[cur_point][next_point] = path_ptr;
                path_lengths_[cur_point][next_point] = path_ptr->length();
                break;
            default:
                path_lengths_[cur_point][next_point] = INFINITY;
                break;
        }
        cost_vector.push_back(path_lengths_[cur_point][next_point]);
    }

    int next_center = cur_center;
    for (int i = 1; i < n_set; i++) {
        cur_center = center_sequence_temp.back();
        center_sequence_temp.pop_back();
        std::vector<double> cost_vector_new;
        std::vector<int> next_point_list;
        for (int j = 0; j < goal_sets[cur_center].size() - 1; j++) {
            if (stop_planning_) {
                return PREEMPTED;
            }
            int cur_point = getIdxInVector(cur_center, j);
            double best = INFINITY;
            int best_idx;
            for (int k = 0; k < goal_sets[next_center].size() - 1; k++) {
                int next_point = getIdxInVector(next_center, k);

                // plan p2p
                PlannerStatus solved = singleGoalPlan(points[cur_point], points[next_point]);
                og::PathGeometric path(ss_->getSpaceInformation());
                std::shared_ptr<og::PathGeometric> path_ptr;
                switch (solved) {
                    case PlannerStatus::SOLVED:
                        path = getSingleSolutionPath();
                        path_ptr = std::make_shared<og::PathGeometric>(path);
                        path_matrix[cur_point][next_point] = path_ptr;
                        path_lengths_[cur_point][next_point] = path_ptr->length();
                        break;
                    default:
                        path_lengths_[cur_point][next_point] = INFINITY;
                        break;
                }

                double cur_cost = path_lengths_[cur_point][next_point] + cost_vector[k];
                if (cur_cost < best) {
                    best = cur_cost;
                    best_idx = k;
                }
            }
            cost_vector_new.push_back(best);
            next_point_list.push_back(best_idx);
        }
        next_point_table[n_set - 1 - i] = next_point_list;
        next_center = cur_center;
        cost_vector.clear();
        cost_vector.insert(cost_vector.end(), cost_vector_new.begin(), cost_vector_new.end());
    }
    // first step from start
    int cur_point = n_points - 1;
    double best = INFINITY;
    int best_idx;
    for (int k = 0; k < goal_sets[next_center].size() - 1; k++) {
        if (stop_planning_) {
            return PREEMPTED;
        }
        int next_point = getIdxInVector(next_center, k);

        // plan p2p
        PlannerStatus solved = singleGoalPlan(points[cur_point], points[next_point]);
        og::PathGeometric path(ss_->getSpaceInformation());
        std::shared_ptr<og::PathGeometric> path_ptr;
        switch (solved) {
            case PlannerStatus::SOLVED:
                path = getSingleSolutionPath();
                path_ptr = std::make_shared<og::PathGeometric>(path);
                path_matrix[cur_point][next_point] = path_ptr;
                path_lengths_[cur_point][next_point] = path_ptr->length();
                break;
            default:
                path_lengths_[cur_point][next_point] = INFINITY;
                break;
        }

        double cur_cost = path_lengths_[cur_point][next_point] + cost_vector[k];
        if (cur_cost < best) {
            best = cur_cost;
            best_idx = k;
        }
    }

    // the first goal is best_idx
    int cur_idx = best_idx;
    std::vector<int> dp_sequence{};

    for (int i = 0; i < n_set; i++) {
        if (i > 0) cur_idx = next_point_table[i - 1][cur_idx];
        int cur_center = center_sequence_[i];
        int next_point = getIdxInVector(cur_center, cur_idx);
        dp_sequence.push_back(next_point);
    }

    int cur_index = n_points - 1;

    for (int next_index : dp_sequence) {
        og::PathGeometricPtr path_ptr_temp = path_matrix[cur_index][next_index];
        og::PathGeometric path_temp = *path_ptr_temp;
        path_buffer_ompl_.push_back(path_ptr_temp);
        complete_path_dp_->append(path_temp);
        cur_index = next_index;
    }
    // close the path
    og::PathGeometricPtr path_ptr_temp = path_matrix[cur_index][n_points - 1];
    og::PathGeometric path_temp = *path_ptr_temp;
    path_buffer_ompl_.push_back(path_ptr_temp);
    complete_path_dp_->append(path_temp);

    // free memory
    for (int i = 0; i < n_points; i++) {
        delete[] path_lengths_[i];
        delete[] path_matrix[i];
    }
    delete[] path_lengths_;
    delete[] path_matrix;

    return PlannerStatus::SOLVED;
}

PlannerStatus Planner::iterativeDP(const std::vector<std::vector<ob::ScopedState<>>>& goal_sets,
                                   std::vector<ob::ScopedState<>> points) {
    ompl::time::point start_misc5 = ompl::time::now();
    int n_points = points.size();
    int n_set = goal_sets.size();
    og::PathGeometricPtr** path_matrix;
    path_lengths_ = new double*[n_points];
    path_matrix = new og::PathGeometricPtr*[n_points];
    for (int i = 0; i < n_points; i++) {
        path_lengths_[i] = new double[n_points];
        path_matrix[i] = new og::PathGeometricPtr[n_points];
    }

    auto center_sequence_temp = center_sequence_;

    double best_cost = INFINITY;
    std::vector<int> best_point_in_set;
    int n_planned_paths = 0;
    double total_dp_time = 0;
    while (true) {
        // start dp
        ompl::time::point dp_start_time = ompl::time::now();
        auto center_sequence_temp = center_sequence_;
        std::vector<double> cost_vector;
        // the last one goal to start

        int cur_center = center_sequence_temp.back();
        center_sequence_temp.pop_back();
        std::vector<std::vector<int>> next_point_table(n_set - 1);
        int next_point = n_points - 1;
        for (int i = 0; i < goal_sets[cur_center].size() - 1; i++) {
            int cur_point = getIdxInVector(cur_center, i);

            if (!path_matrix[cur_point][next_point]) {
                // this means obstacle free path is not planned yet, use euclidean cost instead
                cost_vector.push_back(points[cur_point].distance(points[next_point]));
            } else {
                // this means obstacle free path is planned, use path cost
                cost_vector.push_back(path_lengths_[cur_point][next_point]);
            }
        }

        int next_center = cur_center;
        for (int i = 1; i < n_set; i++) {
            cur_center = center_sequence_temp.back();
            center_sequence_temp.pop_back();
            std::vector<double> cost_vector_new;
            std::vector<int> next_point_list;
            for (int j = 0; j < goal_sets[cur_center].size() - 1; j++) {
                int cur_point = getIdxInVector(cur_center, j);
                double best = INFINITY;
                int best_idx;
                for (int k = 0; k < goal_sets[next_center].size() - 1; k++) {
                    int next_point = getIdxInVector(next_center, k);

                    double cur_cost = 0;
                    if (!path_matrix[cur_point][next_point]) {
                        // this means obstacle free path is not planned yet, use euclidean cost instead
                        cur_cost = points[cur_point].distance(points[next_point]) + cost_vector[k];
                    } else {
                        // this means obstacle free path is planned, use path cost
                        cur_cost = path_lengths_[cur_point][next_point] + cost_vector[k];
                    }

                    if (cur_cost < best) {
                        best = cur_cost;
                        best_idx = k;
                    }
                }
                cost_vector_new.push_back(best);
                next_point_list.push_back(best_idx);
            }
            next_point_table[n_set - 1 - i] = next_point_list;
            next_center = cur_center;
            cost_vector.clear();
            cost_vector.insert(cost_vector.end(), cost_vector_new.begin(), cost_vector_new.end());
        }
        // first step from start

        int cur_point = n_points - 1;
        double best = INFINITY;
        int best_idx;
        for (int k = 0; k < goal_sets[next_center].size() - 1; k++) {
            double cur_cost = 0;
            int next_point = getIdxInVector(next_center, k);
            if (!path_matrix[cur_point][next_point]) {
                // this means obstacle free path is not planned yet, use euclidean cost instead
                cur_cost = points[cur_point].distance(points[next_point]) + cost_vector[k];
            } else {
                // this means obstacle free path is planned, use path cost
                cur_cost = path_lengths_[cur_point][next_point] + cost_vector[k];
            }
            if (cur_cost < best) {
                best = cur_cost;
                best_idx = k;
            }
        }

        // the first goal is best_idx
        int cur_idx = best_idx;
        std::vector<int> point_in_set(n_set);
        for (int i = 0; i < n_set; i++) {
            if (i > 0) cur_idx = next_point_table[i - 1][cur_idx];
            int cur_center = center_sequence_[i];
            point_in_set[cur_center] = getIdxInVector(cur_center, cur_idx);
        }

        double dp_time = ompl::time::seconds(ompl::time::now() - dp_start_time);
        total_dp_time += dp_time;

        // obstacle free path planning according to point_in_set
        std::vector<og::PathGeometric> path_before;
        std::vector<bool> is_planned_one_iteration;
        og::PathGeometric path_after = og::PathGeometric(ss_->getSpaceInformation());
        bool need_to_plan = false;
        double cost = 0.0;
        int last_point = n_points - 1;
        for (int i = 0; i <= n_set; i++) {
            int cur_point;
            if (i == n_set) {
                cur_point = n_points - 1;
            } else {
                int cur_set = center_sequence_[i];

                cur_point = point_in_set[cur_set];
            }
            // check if path is already planned
            if (!path_matrix[last_point][cur_point]) {
                need_to_plan = true;
                n_planned_paths++;
                PlannerStatus solved = singleGoalPlan(points[last_point], points[cur_point]);
                og::PathGeometric path(ss_->getSpaceInformation());
                std::shared_ptr<og::PathGeometric> path_ptr;
                switch (solved) {
                    case PlannerStatus::SOLVED:
                        path = getSingleSolutionPath();
                        path_ptr = std::make_shared<og::PathGeometric>(path);
                        path_matrix[last_point][cur_point] = path_ptr;
                        path_lengths_[last_point][cur_point] = path_ptr->length();
                        break;
                    default:
                        path_lengths_[last_point][cur_point] = INFINITY;
                        break;
                }
                path_after.append(path);
                is_planned_one_iteration.push_back(false);
                path_before.push_back(
                    og::PathGeometric(ss_->getSpaceInformation(), points[last_point].get(), points[cur_point].get()));
            } else {
                path_after.append(*path_matrix[last_point][cur_point]);
                is_planned_one_iteration.push_back(true);
                path_before.push_back(*path_matrix[last_point][cur_point]);
            }
            cost += path_lengths_[last_point][cur_point];
            last_point = cur_point;
        }
        idp_paths_before_plan_.push_back(path_before);
        idp_paths_after_plan_.push_back(path_after);
        is_planned_.push_back(is_planned_one_iteration);
        if (cost < best_cost) {
            best_cost = cost;
            best_point_in_set = point_in_set;
        }
        if (!need_to_plan) {
            break;
        }
    }
    // glue solution
    int cur_index = n_points - 1;
    for (int next_set : center_sequence_) {
        int next_index = best_point_in_set[next_set];
        og::PathGeometricPtr path_ptr_temp = path_matrix[cur_index][next_index];
        og::PathGeometric path_temp = *path_ptr_temp;
        path_buffer_ompl_.push_back(path_ptr_temp);
        complete_path_idp_->append(path_temp);
        cur_index = next_index;
    }
    // close the path
    og::PathGeometricPtr path_ptr_temp = path_matrix[cur_index][n_points - 1];
    og::PathGeometric path_temp = *path_ptr_temp;
    path_buffer_ompl_.push_back(path_ptr_temp);
    complete_path_idp_->append(path_temp);

    // free memory
    for (int i = 0; i < n_points; i++) {
        delete[] path_lengths_[i];
        delete[] path_matrix[i];
    }
    delete[] path_lengths_;
    delete[] path_matrix;
    return PlannerStatus::SOLVED;
}

PlannerStatus Planner::rubberBandAlgorithm(const std::vector<std::vector<ob::ScopedState<>>>& goal_sets,
                                           std::vector<ob::ScopedState<>> points,
                                           std::vector<ob::ScopedState<>> goal_centers) {
    int n_points = points.size();
    int n_set = goal_sets.size();
    og::PathGeometricPtr** path_matrix;
    path_lengths_ = new double*[n_points];
    path_matrix = new og::PathGeometricPtr*[n_points];
    for (int i = 0; i < n_points; i++) {
        path_lengths_[i] = new double[n_points];
        path_matrix[i] = new og::PathGeometricPtr[n_points];
    }

    auto center_sequence_temp = center_sequence_;

    // initialize with euclidean distance to adjacent centers
    std::vector<int> point_in_set(n_set);
    // 0th set, adjacent sets are start and 1st set
    auto center_last = points.back();  // this is the start point
    for (int i = 0; i < n_set; i++) {
        double dist_best = INFINITY;
        auto set_cur = center_sequence_temp[i];
        ob::ScopedState<StateSpace> center_next(ss_->getSpaceInformation());
        if (i == n_set - 1) {
            center_next = points.back();
        } else {
            auto set_next = center_sequence_temp[i + 1];
            center_next = goal_centers[set_next];
        }

        for (int j = 0; j < goal_sets[set_cur].size() - 1; j++) {
            auto point_cur = goal_sets[set_cur][j + 1];
            double dist = point_cur.distance(center_last) + point_cur.distance(center_next);
            if (dist < dist_best) {
                dist_best = dist;
                point_in_set[set_cur] = getIdxInVector(set_cur, j);
            }
        }
        center_last = goal_centers[set_cur];
    }

    // initialization finished, start main RBA
    double best_cost = INFINITY;
    std::vector<int> best_point_in_set;
    int n_planned_paths = 0;
    double total_rba_time = 0;
    while (true) {
        // start locally adjust point in the set
        bool tour_diff_from_last_iter = false;
        bool tour_changed = true;
        ompl::time::point rba_start_time = ompl::time::now();
        while (tour_changed) {
            tour_changed = false;
            // start with the start
            int last_point = n_points - 1;

            // iterate through every set i
            for (int i = 0; i < n_set; i++) {
                int cur_set = center_sequence_temp[i];
                int next_point;
                if (i == n_set - 1) {
                    next_point = n_points - 1;
                } else {
                    int next_set = center_sequence_temp[i + 1];
                    next_point = point_in_set[next_set];
                }

                // iterate through all candidate points in the current set, find the one minimizes the cost from last
                // point plus to the next point;
                double best_local_cost = INFINITY;
                int best_point;
                for (int j = 0; j < goal_sets[cur_set].size() - 1; j++) {
                    auto cur_point = getIdxInVector(cur_set, j);
                    // compute cost
                    double cost = 0;
                    // cost from last to current
                    if (!path_matrix[last_point][cur_point]) {
                        // this means obstacle free path is not planned yet, use euclidean cost instead
                        cost += points[last_point].distance(points[cur_point]);
                    } else {
                        // this means obstacle free path is planned, use path cost
                        cost += path_lengths_[last_point][cur_point];
                    }

                    // cost from current to next;
                    if (!path_matrix[cur_point][next_point]) {
                        // this means obstacle free path is not planned yet, use euclidean cost instead
                        cost += points[cur_point].distance(points[next_point]);
                    } else {
                        // this means obstacle free path is planned, use path cost
                        cost += path_lengths_[cur_point][next_point];
                    }

                    if (cost < best_local_cost) {
                        best_local_cost = cost;
                        best_point = cur_point;
                    }
                }
                if (i < n_set - 1) {
                    if (best_point != point_in_set[cur_set]) {
                        tour_changed = true;
                        tour_diff_from_last_iter = true;
                        point_in_set[cur_set] = best_point;
                    }
                }
                last_point = best_point;
            }
        }
        double rba_time = ompl::time::seconds(ompl::time::now() - rba_start_time);
        total_rba_time += rba_time;

        // obstacle free path planning according to point_in_set
        bool need_to_plan = false;
        double cost = 0.0;
        int last_point = n_points - 1;
        for (int i = 0; i <= n_set; i++) {
            int cur_point;
            if (i == n_set) {
                cur_point = n_points - 1;
            } else {
                int cur_set = center_sequence_temp[i];
                cur_point = point_in_set[cur_set];
            }
            // check if path is already planned
            if (!path_matrix[last_point][cur_point]) {
                need_to_plan = true;
                n_planned_paths++;
                PlannerStatus solved = singleGoalPlan(points[last_point], points[cur_point]);
                og::PathGeometric path(ss_->getSpaceInformation());
                std::shared_ptr<og::PathGeometric> path_ptr;
                switch (solved) {
                    case PlannerStatus::SOLVED:
                        path = getSingleSolutionPath();
                        path_ptr = std::make_shared<og::PathGeometric>(path);
                        path_matrix[last_point][cur_point] = path_ptr;
                        path_lengths_[last_point][cur_point] = path_ptr->length();
                        break;
                    default:
                        path_lengths_[last_point][cur_point] = INFINITY;
                        break;
                }
            }
            cost += path_lengths_[last_point][cur_point];
            last_point = cur_point;
        }
        if (!tour_diff_from_last_iter && !need_to_plan) {
            break;
        }
        if (cost < best_cost) {
            best_cost = cost;
            best_point_in_set = point_in_set;
        }
    }

    // glue solution
    int cur_index = n_points - 1;
    for (int next_set : center_sequence_temp) {
        int next_index = best_point_in_set[next_set];
        og::PathGeometricPtr path_ptr_temp = path_matrix[cur_index][next_index];
        og::PathGeometric path_temp = *path_ptr_temp;
        path_buffer_ompl_.push_back(path_ptr_temp);
        complete_path_rba_->append(path_temp);
        cur_index = next_index;
    }
    // close the path
    og::PathGeometricPtr path_ptr_temp = path_matrix[cur_index][n_points - 1];
    og::PathGeometric path_temp = *path_ptr_temp;
    path_buffer_ompl_.push_back(path_ptr_temp);
    complete_path_rba_->append(path_temp);
    // free memory
    for (int i = 0; i < n_points; i++) {
        delete[] path_lengths_[i];
        delete[] path_matrix[i];
    }
    delete[] path_lengths_;
    delete[] path_matrix;

    return PlannerStatus::SOLVED;
}

std::vector<ob::ScopedState<>> Planner::makeGoalVector(const std::vector<std::vector<ob::ScopedState<>>> goal_sets) {
    std::vector<ob::ScopedState<>> goal_vector;
    for (auto goal_set : goal_sets) {
        // do not append the target center, i.e. the first element of each goal_set
        goal_set.erase(goal_set.begin());
        goal_vector.insert(goal_vector.end(), goal_set.begin(), goal_set.end());
    }
    return goal_vector;
}

PlannerStatus Planner::singleGoalPlan(const ob::ScopedState<>& start, const ob::ScopedState<>& goal,
                                      bool using_all_time, double time) {
    // multi query, do not clear everything, just clear the last query and reset start and goal;
    // remember to clear the last solution, this is done by ss_->setStartAndGoalStates()
    ss_->getPlanner()->clearQuery();
    ss_->setStartAndGoalStates(start, goal);
    ob::PlannerStatus solved;

    try {
        solved = ss_->solve(time);
    }
    // exceptions are not dealt with yet
    catch (ompl::Exception& e) {
        return PlannerStatus::NOT_SOLVED;
    }
    switch (ob::PlannerStatus::StatusType(solved)) {
        case ob::PlannerStatus::EXACT_SOLUTION:
            return PlannerStatus::SOLVED;
        case ob::PlannerStatus::TIMEOUT:
            return PlannerStatus::NOT_SOLVED;
    }
    return PlannerStatus::UNKNOWN;
}

og::PathGeometric Planner::getSingleSolutionPath(const bool& simplyfy) const {
    auto path = ss_->getSolutionPath();
    return path;
}

std::vector<std::vector<ob::ScopedState<>>> Planner::generateSmallCraterTargetSets(int n_set, int n_goal_per_set) {
    std::vector<std::vector<double>> center_list{{-13.3, -9.5}, {7, -11.7},    {7, 8.5},    {10, -8.5}, {-4.3, 1},
                                                 {2.4, -10},    {-13.6, -0.5}, {2.1, -1.2}, {5, -6.3},  {-8.7, 7.6},
                                                 {-1.8, 8.8},   {-11.5, -4.7}, {-8.7, 0.7}};

    std::vector<double> radius_list(13, 1);
    std::vector<std::vector<ob::ScopedState<>>> target_sets;

    for (int i = 0; i < n_set; i++) {
        ob::ScopedState<StateSpace> target_set_center(ss_->getSpaceInformation());
        auto center_temp = center_list[i];

        float height;
        bool has_height = false;
        double r = radius_list[i];
        do {
            // get the height of a point in the vicinity, outside the toi radius, so the height is not on the toi
            double phi = 2 * M_PI * ((rand() / double(RAND_MAX)) - 0.5);
            double dist = r + 0.2 * (rand() / double(RAND_MAX));
            has_height = map_->getMapHeight(
                Eigen::Vector3f(center_temp[0] + dist * cos(phi), center_temp[1] + dist * sin(phi), 0), &height);
        } while (!has_height);

        target_set_center.get()->setXYZ(center_temp[0], center_temp[1], height);
        setSO3FromYaw(target_set_center.get()->rotation(), 0.0);
        OMPL_INFORM("target center : ");
        target_set_center.print();
        std::vector<ob::ScopedState<>> target_set;

        // target center is the first element of each set
        target_set.push_back(target_set_center);
        checker_validate_toi_->addToI(target_set_center, r);
        inclination_motion_validator_validate_toi_->addToI(target_set_center, r);
        r += params_->inspection.distance + params_->robot.torso.length / 2;
        for (int j = 0; j < n_goal_per_set;) {
            // generate angle equidistantly
            // if use traversabilty add random small angle to find valid goal state
            double phi = 2 * M_PI * j / n_goal_per_set - M_PI;
            ob::ScopedState<StateSpace> target(target_set_center);
            double dx = -cos(phi) * r;
            double dy = -sin(phi) * r;
            double target_x = dx + target.get()->getX();
            double target_y = dy + target.get()->getY();
            float height;
            if (!map_->getMapHeight(Eigen::Vector3f(target_x, target_y, 0), &height)) {
                OMPL_WARN(
                    "PoIs including invalid ones (no height), generate emtpy goals (adjust number of poi or the "
                    "inspection distance to have valid PoIs)");
                return {};
            };
            target.get()->setXYZ(target_x, target_y, height);

            setSO3FromYaw(target.get()->rotation(), phi);
            if (target.satisfiesBounds() && checker_->isValid(target)) {
                target_set.push_back(target);
                j++;
            } else {
                OMPL_WARN(
                    "PoIs including invalid ones (validity checker ouputs invalid), generate emtpy goals (adjust "
                    "number of poi or the inspection distance to have valid PoIs)");
                return {};
            }
        }
        target_sets.push_back(target_set);
    }
    return target_sets;
}

ob::ScopedState<> Planner::generateSmallCraterStart() {
    ob::ScopedState<StateSpace> start(ss_->getSpaceInformation());

    double x = -13;
    double y = 8;
    float z = 0.0;

    map_->getMapHeightDefaultInf(Eigen::Vector3f(x, y, 0), &z);
    start.get()->setX(x);
    start.get()->setY(y);
    start.get()->setZ(z);
    bool valid = checker_->isValid(start);
    if (!valid) {
        OMPL_INFORM("!!!!!!!!!! start not valid !!!!!!!");
    }

    setSO3FromYaw(start.get()->rotation(), 0.0);
    return start;
}

ob::ScopedState<> Planner::generateRandomPose() {
    ob::ScopedState<StateSpace> goal(ss_->getSpaceInformation());

    double x = 0.0;
    double y = 0.0;
    float z = 0.0;
    bool valid = false;
    auto bounds = space_->getBounds();
    double x_min = bounds.low[0];
    double y_min = bounds.low[1];
    double x_max = bounds.high[0];
    double y_max = bounds.high[1];
    while (!valid) {
        x = (x_max - x_min) * (rand() / double(RAND_MAX)) + x_min;
        y = (y_max - y_min) * (rand() / double(RAND_MAX)) + y_min;
        map_->getMapHeightDefaultInf(Eigen::Vector3f(x, y, 0), &z);
        goal.get()->setX(x);
        goal.get()->setY(y);
        goal.get()->setZ(z);
        valid = checker_->isValid(goal);
    };

    setSO3FromYaw(goal.get()->rotation(), 2 * M_PI * ((rand() / double(RAND_MAX)) - 0.5));
    return goal;
}

void Planner::setBounds() {
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, params_->planner.state_space.x_lim.low);
    bounds.setHigh(0, params_->planner.state_space.x_lim.high);
    bounds.setLow(1, params_->planner.state_space.y_lim.low);
    bounds.setHigh(1, params_->planner.state_space.y_lim.high);
    bounds.setLow(2, params_->planner.state_space.z_lim.low);
    bounds.setHigh(2, params_->planner.state_space.z_lim.high);

    std::cout << "Lower SE3 position bounds: " << bounds.low[0] << "\t" << bounds.low[1] << "\t" << bounds.low[2]
              << std::endl;
    std::cout << "Upper SE3 position bounds: " << bounds.high[0] << "\t" << bounds.high[1] << "\t" << bounds.high[2]
              << std::endl;

    space_->setBounds(bounds);
}

int Planner::getIdxInVector(const int center_idx, const int pose_idx) {
    return center_idx_list_[center_idx] + pose_idx;
}

void Planner::postProcessPath() {
    for (auto path : path_buffer_ompl_) {
        auto states = path->getStates();
        // from 2. state
        for (int i = 1; i < states.size() - 1; i++) {
            auto cur_state_ptr = states[i]->as<Planner::StateType>();
            auto next_state_ptr = states[i + 1]->as<Planner::StateType>();
            // compute yaw,
            double yaw =
                atan2(next_state_ptr->getY() - cur_state_ptr->getY(), next_state_ptr->getX() - cur_state_ptr->getX());
            // set yaw
            setSO3FromYaw(cur_state_ptr->rotation(), yaw);
        }
    }
}