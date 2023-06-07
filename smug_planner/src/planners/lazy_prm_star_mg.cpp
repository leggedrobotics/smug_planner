// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#include "smug_planner/planners/lazy_prm_star_mg.h"

#include <ompl/base/Planner.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/util/Exception.h>

#include <algorithm>
#include <boost/foreach.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/graph/lookup_edge.hpp>
#include <chrono>

#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/geometric/planners/prm/LazyPRM.h"
#include "ompl/tools/config/SelfConfig.h"

#ifndef OMPL_GEOMETRIC_PLANNERS_PRM_A_STAR_GOAL_VISITOR_
#define OMPL_GEOMETRIC_PLANNERS_PRM_A_STAR_GOAL_VISITOR_

#include <boost/graph/astar_search.hpp>

#include "smug_planner/motion_validator/inclination_motion_validator_validate_toi.h"
#include "smug_planner/sampler.h"
#include "smug_planner/validity_checker/validity_checker_validate_toi.h"

namespace {
struct AStarFoundGoal {};  // exception for termination

// visitor that terminates when we find the goal
// V is the vertex type
template <typename V>
class AStarGoalVisitor : public boost::default_astar_visitor {
   public:
    AStarGoalVisitor(const V &goal) : goal_(goal) {}

    // G is the graph type
    template <typename G>
    void examine_vertex(const V &u, const G & /*unused*/) {
        if (u == goal_) throw AStarFoundGoal();
    }

   private:
    V goal_;
};
}  // namespace

#endif

namespace ompl {
namespace magic {
/** \brief The number of nearest neighbors to consider by
    default in the construction of the PRM roadmap */
static const unsigned int DEFAULT_NEAREST_NEIGHBORS_LAZY = 5;

/** \brief When optimizing solutions with lazy planners, this is the minimum
    number of path segments to add before attempting a new optimized solution
    extraction */
static const unsigned int MIN_ADDED_SEGMENTS_FOR_LAZY_OPTIMIZATION = 5;
}  // namespace magic
}  // namespace ompl

using namespace smug_planner;

LazyPRMStarMG::LazyPRMStarMG(const ob::SpaceInformationPtr &si, const ParamsConstPtr &params) : og::LazyPRMstar(si) {
    params_ = params;
    max_stall_ = params->planner.max_stall;
}

og::LazyPRM::Vertex LazyPRMStarMG::addMilestone(ob::State *state) {
    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    vertexValidityProperty_[m] = VALIDITY_TRUE;
    unsigned long int newComponent = componentCount_++;
    vertexComponentProperty_[m] = newComponent;
    componentSize_[newComponent] = 1;
    if (params_->debug.visualize_vertex_validity_type) {
        n_vertex_++;
        all_vertex_index_[m] = n_vertex_;
        all_vertex_state_.push_back(si_->cloneState(state)->as<ob::SE3StateSpace::StateType>());
        all_vertex_validity_type_.push_back(ValidityType::UNCHECKED);
    }

    // Which milestones will we attempt to connect to?
    const std::vector<Vertex> &neighbors = connectionStrategy_(m);
    for (Vertex n : neighbors) {
        if (connectionFilter_(m, n)) {
            const ob::Cost weight = opt_->motionCost(stateProperty_[m], stateProperty_[n]);
            const Graph::edge_property_type properties(weight);
            const Edge &e = boost::add_edge(m, n, properties, g_).first;
            edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
            uniteComponents(m, n);
            if (params_->debug.visualize_edge_validity_type) {
                n_edge_++;
                all_edge_index_[e] = n_edge_;
                all_edge_state_.push_back(std::pair<ob::SE3StateSpace::StateType *, ob::SE3StateSpace::StateType *>(
                    si_->cloneState(stateProperty_[m])->as<ob::SE3StateSpace::StateType>(),
                    si_->cloneState(stateProperty_[n])->as<ob::SE3StateSpace::StateType>()));
                all_edge_validity_type_.push_back(ValidityType::UNCHECKED);
            }
        }
    }
    nn_->add(m);

    return m;
}

og::LazyPRM::Vertex LazyPRMStarMG::addMilestoneWithinToI(ob::State *state) {
    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    vertexValidityProperty_[m] = VALIDITY_MG_WITHIN_TOI;
    unsigned long int newComponent = componentCount_++;
    vertexComponentProperty_[m] = newComponent;
    componentSize_[newComponent] = 1;
    if (params_->debug.visualize_vertex_validity_type) {
        n_vertex_++;
        all_vertex_index_[m] = n_vertex_;
        all_vertex_state_.push_back(si_->cloneState(state)->as<ob::SE3StateSpace::StateType>());
        all_vertex_validity_type_.push_back(ValidityType::UNCHECKED);
    }

    // Which milestones will we attempt to connect to?
    const std::vector<Vertex> &neighbors = connectionStrategy_(m);
    for (Vertex n : neighbors) {
        if (connectionFilter_(m, n)) {
            const ob::Cost weight = opt_->motionCost(stateProperty_[m], stateProperty_[n]);
            const Graph::edge_property_type properties(weight);
            const Edge &e = boost::add_edge(m, n, properties, g_).first;
            edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
            uniteComponents(m, n);
            if (params_->debug.visualize_edge_validity_type) {
                n_edge_++;
                all_edge_index_[e] = n_edge_;
                all_edge_state_.push_back(std::pair<ob::SE3StateSpace::StateType *, ob::SE3StateSpace::StateType *>(
                    si_->cloneState(stateProperty_[m])->as<ob::SE3StateSpace::StateType>(),
                    si_->cloneState(stateProperty_[n])->as<ob::SE3StateSpace::StateType>()));
                all_edge_validity_type_.push_back(ValidityType::UNCHECKED);
            }
        }
    }
    nn_->add(m);

    return m;
}

ob::PlannerStatus LazyPRMStarMG::solve(const ob::PlannerTerminationCondition &ptc) {
    switch (method_) {
        case SOLVE_MG_INFORMED:
            return solveInformed(ptc);
        case SOLVE_MG_GET:
            return solveGet(ptc);
        default:
            return solveInformed(ptc);
    }
}

ob::PlannerStatus LazyPRMStarMG::solveGet(const ob::PlannerTerminationCondition &ptc) {
    // directly get the solution
    auto *goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());

    // Add the valid start states as milestones
    while (const ob::State *st = pis_.nextStart()) startM_.push_back(addMilestone(si_->cloneState(st)));

    if (!goal->couldSample()) {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return ob::PlannerStatus::INVALID_GOAL;
    }

    // Ensure there is at least one valid goal state
    if (goal->maxSampleCount() > goalM_.size() || goalM_.empty()) {
        const ob::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
        if (st != nullptr) {
            goalM_.push_back(addMilestone(si_->cloneState(st)));
        }
        if (goalM_.empty()) {
            OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
            return ob::PlannerStatus::INVALID_GOAL;
        }
    }

    unsigned long int nrStartStates = boost::num_vertices(g_);
    OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);

    bestCost_ = opt_->infiniteCost();
    ob::State *workState = si_->allocState();
    std::pair<std::size_t, std::size_t> startGoalPair;
    ob::PathPtr bestSolution;
    bool fullyOptimized = false;

    // do not grow map, assume map is complete

    Vertex startV = startM_[startGoalPair.first];
    Vertex goalV = goalM_[startGoalPair.second];
    ob::PathPtr solution;
    do {
        solution = constructSolution(startV, goalV);
    } while (!solution && vertexComponentProperty_[startV] == vertexComponentProperty_[goalV]);
    if (solution) {
        ob::Cost c = solution->cost(opt_);
        if (opt_->isSatisfied(c)) {
            fullyOptimized = true;
            bestSolution = solution;
            bestCost_ = c;
        }
        if (opt_->isCostBetterThan(c, bestCost_)) {
            bestSolution = solution;
            bestCost_ = c;
        }
    }

    si_->freeState(workState);

    if (bestSolution) {
        ob::PlannerSolution psol(bestSolution);
        psol.setPlannerName(getName());
        // if the solution was optimized, we mark it as such
        psol.setOptimized(opt_, bestCost_, fullyOptimized);
        pdef_->addSolutionPath(psol);
    }

    OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);

    return bestSolution ? ob::PlannerStatus::EXACT_SOLUTION : ob::PlannerStatus::TIMEOUT;
}

ob::PlannerStatus LazyPRMStarMG::solveInformed(const ob::PlannerTerminationCondition &ptc) {
    checkValidity();
    auto informed_sampler = std::dynamic_pointer_cast<smug_planner::InformedSampler>(sampler_);
    auto *goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr) {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return ob::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // Add the valid start states as milestones
    while (const ob::State *st = pis_.nextStart()) {
        if (stage_ == 1) {
            startM_.push_back(addMilestoneWithinToI(si_->cloneState(st)));
        } else {
            startM_.push_back(addMilestone(si_->cloneState(st)));
        }
    }

    if (startM_.empty()) {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return ob::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample()) {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return ob::PlannerStatus::INVALID_GOAL;
    }

    // Ensure there is at least one valid goal state
    if (goal->maxSampleCount() > goalM_.size() || goalM_.empty()) {
        const ob::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
        if (st != nullptr) {
            if (stage_ == 1) {
                goalM_.push_back(addMilestoneWithinToI(si_->cloneState(st)));
            } else {
                goalM_.push_back(addMilestone(si_->cloneState(st)));
            }
        }
        if (goalM_.empty()) {
            OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
            return ob::PlannerStatus::INVALID_GOAL;
        }
    }

    unsigned long int nrStartStates = boost::num_vertices(g_);

    bestCost_ = opt_->infiniteCost();

    ob::State *workState = si_->allocState();
    std::pair<std::size_t, std::size_t> startGoalPair;
    ob::PathPtr bestSolution;
    bool fullyOptimized = false;
    bool someSolutionFound = false;
    unsigned int optimizingComponentSegments = 0;
    bool optimized = false;
    int solutionFoundTimes = 0;
    int improvedTimes = 0;
    // Grow roadmap in lazy fashion -- add vertices and edges without checking validity
    int stall = 0;

    while (!someSolutionFound || !optimized) {
        int n_sample = 0;

        Vertex start_vertex = startM_[startGoalPair.first];
        Vertex goal_vertex = goalM_[startGoalPair.second];

        auto start_state = stateProperty_[start_vertex];
        auto goal_state = stateProperty_[goal_vertex];
        ValidityType validity_type;
        Vertex addedVertex;
        do {
            n_sample++;

            bool has_sample =
                informed_sampler->sampleUniformInformed(workState, start_state, goal_state, bestCost_.value());

            if (!has_sample) {
                optimized = true;
                break;
            }

            if (stage_ == 1) {
                validity_type =
                    std::dynamic_pointer_cast<StateValidityCheckerValidateToI>(si_->getStateValidityChecker())
                        ->validityType(workState);
                if (validity_type == ValidityType::VALID || validity_type == ValidityType::TRAVERSABILITY_HIGH) {
                    addedVertex = addMilestone(si_->cloneState(workState));
                    break;
                }
                if (validity_type == ValidityType::WITHIN_TOI) {
                    addedVertex = addMilestoneWithinToI(si_->cloneState(workState));
                    break;
                }
            } else {
                validity_type = std::dynamic_pointer_cast<StateValidityChecker>(si_->getStateValidityChecker())
                                    ->validityType(workState);
                if (validity_type == ValidityType::VALID || validity_type == ValidityType::TRAVERSABILITY_HIGH) {
                    addedVertex = addMilestone(si_->cloneState(workState));
                    break;
                }
            }

        } while (true);
        if (optimized) {
            // only if the informed sampling found out the solution is already optimal
            break;
        }

        const long int solComponent = solutionComponent(&startGoalPair);
        // If the start & goal are connected and we either did not find any solution
        // so far or the one we found still needs optimizing and we just added an edge
        // to the connected component that is used for the solution, we attempt to
        // construct a new solution.

        if (solComponent != -1 &&
            (!someSolutionFound || (long int)vertexComponentProperty_[addedVertex] == solComponent)) {
            // If we already have a solution, we are optimizing. We check that we added at least
            // a few segments to the connected component that includes the previously found
            // solution before attempting to construct a new solution.
            if (someSolutionFound) {
                if (++optimizingComponentSegments < ompl::magic::MIN_ADDED_SEGMENTS_FOR_LAZY_OPTIMIZATION) {
                    continue;
                }
                optimizingComponentSegments = 0;
            }
            Vertex startV = startM_[startGoalPair.first];
            Vertex goalV = goalM_[startGoalPair.second];
            ob::PathPtr solution;
            ompl::time::point start = ompl::time::now();
            do {
                solution = constructSolution(startV, goalV);
            } while (!solution && vertexComponentProperty_[startV] == vertexComponentProperty_[goalV]);

            if (solution) {
                solutionFoundTimes++;
                someSolutionFound = true;
                ob::Cost c = solution->cost(opt_);
                if (opt_->isSatisfied(c)) {
                    fullyOptimized = true;
                    bestSolution = solution;
                    bestCost_ = c;
                    break;
                }
                if (c.value() > bestCost_.value() * 0.98) {
                    stall += 1;
                    if (stall >= max_stall_) {
                        optimized = true;
                    }
                } else {
                    stall = 0;
                }

                if (opt_->isCostBetterThan(c, bestCost_)) {
                    improvedTimes++;
                    bestSolution = solution;
                    bestCost_ = c;
                }
            }
        }
    }
    si_->freeState(workState);

    if (bestSolution) {
        ob::PlannerSolution psol(bestSolution);
        psol.setPlannerName(getName());
        // if the solution was optimized, we mark it as such
        psol.setOptimized(opt_, bestCost_, fullyOptimized);
        pdef_->addSolutionPath(psol);
    }
    return bestSolution ? ob::PlannerStatus::EXACT_SOLUTION : ob::PlannerStatus::TIMEOUT;
}

ob::PathPtr LazyPRMStarMG::constructSolution(const Vertex &start, const Vertex &goal) {
    // Need to update the index map here, becuse nodes may have been removed and
    // the numbering will not be 0 .. N-1 otherwise.

    unsigned long int index = 0;
    boost::graph_traits<Graph>::vertex_iterator vi, vend;
    for (boost::tie(vi, vend) = boost::vertices(g_); vi != vend; ++vi, ++index) indexProperty_[*vi] = index;

    boost::property_map<Graph, boost::vertex_predecessor_t>::type prev;
    try {
        // Consider using a persistent distance_map if it's slow
        boost::astar_search(
            g_, start, [this, goal](Vertex v) { return costHeuristic(v, goal); },
            boost::predecessor_map(prev)
                .distance_compare([this](ob::Cost c1, ob::Cost c2) { return opt_->isCostBetterThan(c1, c2); })
                .distance_combine([this](ob::Cost c1, ob::Cost c2) { return opt_->combineCosts(c1, c2); })
                .distance_inf(opt_->infiniteCost())
                .distance_zero(opt_->identityCost())
                .visitor(AStarGoalVisitor<Vertex>(goal)));
    } catch (AStarFoundGoal &) {
    }

    if (prev[goal] == goal) throw ompl::Exception(name_, "Could not find solution path");

    // First, get the solution states without copying them, and check them for validity.
    // We do all the node validity checks for the vertices, as this may remove a larger
    // part of the graph (compared to removing an edge).
    std::vector<const ob::State *> states(1, stateProperty_[goal]);
    std::set<Vertex> milestonesToRemove;
    for (Vertex pos = prev[goal]; prev[pos] != pos; pos = prev[pos]) {
        const ob::State *st = stateProperty_[pos];
        unsigned int &vd = vertexValidityProperty_[pos];
        if (stage_ == 2) {
            // WITHIN_TOI is unkown state and needs to be update
            if (vd == VALIDITY_MG_UNKNOWN || vd == VALIDITY_MG_WITHIN_TOI) {
                auto validity_checker = std::dynamic_pointer_cast<StateValidityChecker>(si_->getStateValidityChecker());
                ValidityType validity_type = validity_checker->validityType(st);
                if (params_->debug.visualize_vertex_validity_type) {
                    unsigned long int index = all_vertex_index_[pos];
                    all_vertex_validity_type_[index] = validity_type;
                }

                switch (validity_type) {
                    case ValidityType::VALID:
                        vd = VALIDITY_MG_TRUE;
                        break;
                    case ValidityType::NO_HEIGHT:
                        vd = VALIDITY_MG_NO_HEIGHT;
                        break;
                    case ValidityType::COLLISION:
                        vd = VALIDITY_MG_COLLISION;
                        break;
                    case ValidityType::TRAVERSABILITY_LOW:
                        vd = VALIDITY_MG_TRAVERSABILITY_LOW;
                        break;
                    case ValidityType::TRAVERSABILITY_HIGH:
                        vd = VALIDITY_MG_TRAVERSABILITY_HIGH;
                        break;
                    default:
                        break;
                }
            }
        }
        if (vd == VALIDITY_MG_COLLISION || vd == VALIDITY_MG_TRAVERSABILITY_LOW || vd == VALIDITY_MG_NO_HEIGHT)
            milestonesToRemove.insert(pos);
        if (milestonesToRemove.empty()) states.push_back(st);
    }

    // We remove *all* invalid vertices. This is not entirely as described in the original LazyPRM
    // paper, as the paper suggest removing the first vertex only, and then recomputing the
    // shortest path. Howeve, the paper says the focus is on efficient vertex & edge removal,
    // rather than collision checking, so this modification is in the spirit of the paper.
    if (!milestonesToRemove.empty()) {
        // OMPL_INFORM("need to remove invalid state");
        unsigned long int comp = vertexComponentProperty_[start];
        // Remember the current neighbors.
        std::set<Vertex> neighbors;
        for (auto it = milestonesToRemove.begin(); it != milestonesToRemove.end(); ++it) {
            boost::graph_traits<Graph>::adjacency_iterator nbh, last;
            for (boost::tie(nbh, last) = boost::adjacent_vertices(*it, g_); nbh != last; ++nbh)
                if (milestonesToRemove.find(*nbh) == milestonesToRemove.end()) neighbors.insert(*nbh);
            // Remove vertex from nearest neighbors data structure.
            nn_->remove(*it);
            // Free vertex state.
            si_->freeState(stateProperty_[*it]);
            // Remove all edges.
            boost::clear_vertex(*it, g_);
            // Remove the vertex.
            boost::remove_vertex(*it, g_);
        }
        // Update the connected component ID for neighbors.
        for (auto neighbor : neighbors) {
            if (comp == vertexComponentProperty_[neighbor]) {
                unsigned long int newComponent = componentCount_++;
                componentSize_[newComponent] = 0;
                markComponent(neighbor, newComponent);
            }
        }

        return ob::PathPtr();
    }

    // start is checked for validity already
    states.push_back(stateProperty_[start]);

    // Check the edges too, if the vertices were valid. Remove the first invalid edge only.
    std::vector<const ob::State *>::const_iterator prevState = states.begin(), state = prevState + 1;
    Vertex prevVertex = goal, pos = prev[goal];
    int n_edge = 0;
    do {
        n_edge++;
        Edge e = boost::lookup_edge(pos, prevVertex, g_).first;
        unsigned int &evd = edgeValidityProperty_[e];
        bool need_edge_checking;
        if (stage_ == 1) {
            need_edge_checking = evd == VALIDITY_MG_UNKNOWN;
        } else {
            need_edge_checking = (evd == VALIDITY_MG_UNKNOWN) || (evd == VALIDITY_MG_WITHIN_TOI);
        }
        if (need_edge_checking) {
            ompl::time::point start = ompl::time::now();
            ValidityType edge_validity_type;
            if (stage_ == 1) {
                edge_validity_type =
                    std::dynamic_pointer_cast<InclinationMotionValidatorValidateToI>(si_->getMotionValidator())
                        ->validityType(*state, *prevState);
                switch (edge_validity_type) {
                    case ValidityType::VALID:
                        evd = VALIDITY_MG_TRUE;
                        break;
                    case ValidityType::WITHIN_TOI:
                        evd = VALIDITY_MG_WITHIN_TOI;
                        break;
                    default:
                        evd = VALIDITY_MG_COLLISION;
                        break;
                }
            } else {
                bool motion_valid = si_->checkMotion(*state, *prevState);
                if (motion_valid) {
                    edge_validity_type = ValidityType::VALID;
                    evd = VALIDITY_MG_TRUE;
                } else {
                    edge_validity_type = ValidityType::COLLISION;
                    evd = VALIDITY_MG_COLLISION;
                }
            }
            // edge_validity_type = valid, within toi, collision(including low traversability)

            if (params_->debug.visualize_edge_validity_type) {
                unsigned long int edge_index = all_edge_index_[e];
                all_edge_validity_type_[edge_index] = edge_validity_type;
            }

            double stop = ompl::time::seconds(ompl::time::now() - start);
        }
        // remove invalid edge after checking
        if (evd == VALIDITY_MG_COLLISION) {
            boost::remove_edge(e, g_);
            unsigned long int newComponent = componentCount_++;
            componentSize_[newComponent] = 0;
            markComponent(pos, newComponent);
            return ob::PathPtr();
        }
        prevState = state;
        ++state;
        prevVertex = pos;
        pos = prev[pos];
    } while (prevVertex != pos);

    auto p(std::make_shared<og::PathGeometric>(si_));
    for (std::vector<const ob::State *>::const_reverse_iterator st = states.rbegin(); st != states.rend(); ++st)
        p->append(*st);

    return p;
}

void LazyPRMStarMG::setStage(const int stage) { stage_ = stage; }

void LazyPRMStarMG::setSolveMethod(const int method) { method_ = method; }

boost::property_map<og::LazyPRM::Graph, og::LazyPRM::vertex_flags_t>::type LazyPRMStarMG::getVertexValidityProperty() {
    return vertexValidityProperty_;
}

boost::property_map<og::LazyPRM::Graph, og::LazyPRM::edge_flags_t>::type LazyPRMStarMG::getEdgeValidityProperty() {
    return edgeValidityProperty_;
}

std::vector<ob::SE3StateSpace::StateType *> LazyPRMStarMG::getAllVertexState() { return all_vertex_state_; }

std::vector<ValidityType> LazyPRMStarMG::getAllVertexValidity() { return all_vertex_validity_type_; }

std::vector<std::pair<ob::SE3StateSpace::StateType *, ob::SE3StateSpace::StateType *>>
LazyPRMStarMG::getAllEdgeState() {
    return all_edge_state_;
}

std::vector<ValidityType> LazyPRMStarMG::getAllEdgeValidity() { return all_edge_validity_type_; }

void LazyPRMStarMG::clearGraphValidity() {
    n_vertex_ = 0;
    n_edge_ = 0;
    // free memory
    for (auto state : all_vertex_state_) {
        si_->freeState(state);
    }
    all_vertex_state_.clear();
    all_vertex_validity_type_.clear();

    for (auto state : all_edge_state_) {
        si_->freeState(state.first);
        si_->freeState(state.second);
    }
    all_edge_state_.clear();
    all_edge_validity_type_.clear();
    all_edge_index_.clear();
}

void LazyPRMStarMG::clear() {
    clearGraphValidity();
    og::LazyPRMstar::clear();
}