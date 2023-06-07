// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#pragma once

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>

#include <atomic>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include "smug_planner/params.h"
#include "smug_planner/validity_type.h"

namespace og = ompl::geometric;
namespace ob = ompl::base;

namespace smug_planner {

class LazyPRMStarMG : public og::LazyPRMstar {
   protected:
    ParamsConstPtr params_;
    int max_stall_{3};
    int stage_{1};  // 1 or 2
    int method_{SOLVE_MG_INFORMED};
    std::vector<ob::SE3StateSpace::StateType *> all_vertex_state_;  // a vector containing state of every added vertex
    std::vector<ValidityType> all_vertex_validity_type_;  // a vector containing validity type of every added vertex
    boost::property_map<Graph, boost::vertex_index_t>::type all_vertex_index_;  // map from vertex to index
    unsigned long int n_vertex_{0};                                             // number of all added vertices

    std::vector<std::pair<ob::SE3StateSpace::StateType *, ob::SE3StateSpace::StateType *>>
        all_edge_state_;                                // a vector containing state of two ends of every added edge
    std::vector<ValidityType> all_edge_validity_type_;  // a vector containing validity type of every added edge
    std::map<Edge, unsigned long int> all_edge_index_;  // map from edge to edge
    unsigned long int n_edge_{0};                       // number of all added edges

   public:
    using LazyPRMstar::LazyPRMstar;

    LazyPRMStarMG(const ob::SpaceInformationPtr &si, const ParamsConstPtr &params);

    // public vallidity flags for debug visualization, accessible by outside

    /** \brief Flag indicating validity of an edge or a vertex */
    static const unsigned int VALIDITY_MG_UNKNOWN = 0;
    /** \brief Flag indicating validity of an edge or a vertex */
    static const unsigned int VALIDITY_MG_TRUE = 1;
    /** \brief Flag indicating validity of an edge or a vertex */
    static const unsigned int VALIDITY_MG_NO_HEIGHT = 2;
    /** \brief Flag indicating validity of an edge or a vertex */
    static const unsigned int VALIDITY_MG_COLLISION = 3;
    /** \brief Flag indicating validity of an edge or a vertex */
    static const unsigned int VALIDITY_MG_TRAVERSABILITY_LOW = 4;
    /** \brief Flag indicating validity of an edge or a vertex */
    static const unsigned int VALIDITY_MG_TRAVERSABILITY_HIGH = 5;
    /** \brief Flag indicating validity of an edge or a vertex */
    static const unsigned int VALIDITY_MG_WITHIN_TOI = 6;

    virtual ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;

    Vertex addMilestone(ob::State *state);

    Vertex addMilestoneWithinToI(ob::State *state);

    ob::PlannerStatus solveInformed(const ob::PlannerTerminationCondition &ptc);

    ob::PlannerStatus solveGet(const ob::PlannerTerminationCondition &ptc);

    ob::PathPtr constructSolution(const Vertex &start, const Vertex &goal);

    void setStage(const int stage);

    // get validity property for the visualizer
    boost::property_map<og::LazyPRM::Graph, og::LazyPRM::vertex_flags_t>::type getVertexValidityProperty();

    boost::property_map<og::LazyPRM::Graph, og::LazyPRM::edge_flags_t>::type getEdgeValidityProperty();

    std::vector<ob::SE3StateSpace::StateType *> getAllVertexState();
    std::vector<ValidityType> getAllVertexValidity();

    std::vector<std::pair<ob::SE3StateSpace::StateType *, ob::SE3StateSpace::StateType *>> getAllEdgeState();
    std::vector<ValidityType> getAllEdgeValidity();

    // public method flags for debug, accessible by outside
    static const unsigned int SOLVE_MG_INFORMED = 0;
    static const unsigned int SOLVE_MG_GET = 1;

    void setSolveMethod(const int method);

    void clearGraphValidity();

    void clear() override;
};

}  // namespace smug_planner