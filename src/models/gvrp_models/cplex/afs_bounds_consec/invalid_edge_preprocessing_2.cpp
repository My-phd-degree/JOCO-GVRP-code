#include "models/gvrp_models/cplex/afs_bounds_consec/afs_bounds_consec.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/preprocessing.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/invalid_edge_preprocessing_2.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::afs_bounds_consec;

Invalid_edge_preprocessing_2::Invalid_edge_preprocessing_2 (Afs_bounds_consec& afs_bounds_consec) : Preprocessing (afs_bounds_consec) {
  if (afs_bounds_consec.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing_2::add () {
  list<pair<int, int>> edges = get_invalid_edges_2(afs_bounds_consec.instance, *afs_bounds_consec.gvrp_afs_tree);
  afs_bounds_consec.nPreprocessings2 = edges.size();
  for (const auto& [i, j] : edges) {
    if (i == afs_bounds_consec.instance.depot.id)
      afs_bounds_consec.model.add(afs_bounds_consec.x[0][afs_bounds_consec.customersC0Indexes[j]] == 0);
    else
      afs_bounds_consec.model.add(afs_bounds_consec.x[afs_bounds_consec.customersC0Indexes[i]][0] == 0);
  }
}
