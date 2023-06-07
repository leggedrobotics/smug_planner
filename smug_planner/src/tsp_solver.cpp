// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#include "smug_planner/tsp_solver.h"

using namespace smug_planner;

TSP_Solver::TSP_Solver(tsp_solver::DataModel data) : data_(std::move(data)) {
    manager_ = std::make_unique<RoutingIndexManager>(data_.distance_matrix.size(), data_.num_vehicles, data_.depot);
    routing_ = std::make_unique<RoutingModel>(*manager_);
}

std::vector<int> TSP_Solver::solve() {
    const int transit_callback_index =
        routing_->RegisterTransitCallback([this](int64_t from_index, int64_t to_index) -> int64_t {
            // Convert from routing variable Index to distance matrix NodeIndex.
            auto from_node = manager_->IndexToNode(from_index).value();
            auto to_node = manager_->IndexToNode(to_index).value();
            return data_.distance_matrix[from_node][to_node];
        });

    routing_->SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

    RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
    searchParameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);

    const Assignment* solution = routing_->SolveWithParameters(searchParameters);
    std::vector<int> routes;

    // Get routes.
    int idx = routing_->Start(0);
    routes.push_back(manager_->IndexToNode(idx).value());
    int j = 0;
    while (!routing_->IsEnd(idx)) {
        std::cout << j << std::endl;
        j++;
        idx = solution->Value(routing_->NextVar(idx));
        routes.push_back(manager_->IndexToNode(idx).value());
        std::cout << manager_->IndexToNode(idx).value() << std::endl;
    }
    routes.erase(routes.begin());
    routes.pop_back();

    return routes;
}