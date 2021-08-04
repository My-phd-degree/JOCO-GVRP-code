#include "models/gvrp_models/cplex/afs_bounds_consec/afs_bounds_consec.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/preprocessing.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/invalid_edge_preprocessing_3.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>
#include <string>

using namespace std;
using namespace utils;
using namespace models::gvrp_models::cplex::afs_bounds_consec;

Invalid_edge_preprocessing_3::Invalid_edge_preprocessing_3 (Afs_bounds_consec& afs_bounds_consec) : Preprocessing (afs_bounds_consec) {
  if (afs_bounds_consec.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing_3::add () {
  list<pair<int, int>> edges = get_invalid_edges_3 (afs_bounds_consec.instance, *afs_bounds_consec.gvrp_afs_tree);
  afs_bounds_consec.nPreprocessings3 = edges.size();
  for (const auto& [i, j] : edges) {
    afs_bounds_consec.model.add(afs_bounds_consec.x[afs_bounds_consec.customersC0Indexes[i]][afs_bounds_consec.customersC0Indexes[j]] == 0);
  }
}
