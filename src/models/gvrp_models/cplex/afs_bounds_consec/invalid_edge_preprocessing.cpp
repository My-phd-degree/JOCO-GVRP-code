#include "models/gvrp_models/cplex/afs_bounds_consec/afs_bounds_consec.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/preprocessing.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/invalid_edge_preprocessing.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::afs_bounds_consec;

Invalid_edge_preprocessing::Invalid_edge_preprocessing (Afs_bounds_consec& afs_bounds_consec) : Preprocessing (afs_bounds_consec) {
  if (afs_bounds_consec.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing::add () {
  //invalid edges
  list<pair<int, int>> edges = get_invalid_edges_1(afs_bounds_consec.instance);
  afs_bounds_consec.nPreprocessings1 = 0;
  for (auto const& [i, j] : edges) {
    auto c1 = afs_bounds_consec.customersC0Indexes.find(i),
         c2 = afs_bounds_consec.customersC0Indexes.find(j);
    auto f1 = afs_bounds_consec.afss_FIndexes.find(i),
         f2 = afs_bounds_consec.afss_FIndexes.find(j);
    //both are customers
    if (c1 != afs_bounds_consec.customersC0Indexes.end() && c2 != afs_bounds_consec.customersC0Indexes.end()) {
      afs_bounds_consec.model.add(afs_bounds_consec.x[c1->second][c2->second] == 0);
      ++afs_bounds_consec.nPreprocessings1;
    }
    //first is afs and second is customer
    if (f1 != afs_bounds_consec.afss_FIndexes.end() && c2 != afs_bounds_consec.customersC0Indexes.end()) {
      for (int i = 0; i < afs_bounds_consec.c0.size(); ++i)
        for (int f = 0; f < afs_bounds_consec._f.size(); ++f) {
          afs_bounds_consec.model.add(afs_bounds_consec.y[i][f][f1->second][c2->second] == 0);
          ++afs_bounds_consec.nPreprocessings1;
        }
    }
    //first is customer and second is afs 
    if (c1 != afs_bounds_consec.customersC0Indexes.end() && f2 != afs_bounds_consec.afss_FIndexes.end()) {
      for (int i = 0; i < afs_bounds_consec.c0.size(); ++i)
        for (int f = 0; f < afs_bounds_consec._f.size(); ++f) {
          afs_bounds_consec.model.add(afs_bounds_consec.y[c1->second][f2->second][f][i] == 0);
          ++afs_bounds_consec.nPreprocessings1;
        }
    }
  }
}
