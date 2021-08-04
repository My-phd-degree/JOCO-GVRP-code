#include "models/gvrp_models/cplex/matheus_model_4/matheus_model_4.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/invalid_edge_preprocessing_2.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::matheus_model_4;

Invalid_edge_preprocessing_2::Invalid_edge_preprocessing_2 (Matheus_model_4& matheus_model_4) : Preprocessing (matheus_model_4) {
  if (matheus_model_4.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing_2::add () {
  list<pair<int, int>> edges = get_invalid_edges_2(matheus_model_4.instance, *matheus_model_4.gvrp_afs_tree);
  matheus_model_4.nPreprocessings2 = edges.size();
  for (const auto& [i, j] : edges) {
    if (i == matheus_model_4.instance.depot.id)
      matheus_model_4.model.add(matheus_model_4.x[0][matheus_model_4.customersC0Indexes[j]] == 0);
    else
      matheus_model_4.model.add(matheus_model_4.x[matheus_model_4.customersC0Indexes[i]][0] == 0);
  }
}
