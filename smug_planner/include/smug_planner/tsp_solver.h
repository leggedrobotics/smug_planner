// ======================================================================
// Copyright (c) 2023 Changan Chen
// Robotic Systems Lab, ETH Zurich
// All rights reserved.
// This source code is licensed under the MIT license.
// See LICENSE file in the project root for details.
// ======================================================================

#pragma once
#include <vector>

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

using namespace operations_research;

namespace smug_planner {

namespace tsp_solver {
struct DataModel;
}

struct tsp_solver::DataModel {
    std::vector<std::vector<double>> distance_matrix;
    int num_vehicles = 1;
    RoutingIndexManager::NodeIndex depot{0};
};

class TSP_Solver {
   public:
    std::vector<int> solve();
    TSP_Solver(tsp_solver::DataModel data);

   protected:
    tsp_solver::DataModel data_;
    std::unique_ptr<RoutingIndexManager> manager_;
    std::unique_ptr<RoutingModel> routing_;
};

}  // namespace smug_planner