#include "models/gvrp_models/cplex/lh_model/lh_model.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/lh_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/lh_model/invalid_edge_preprocessing.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::lh_model;

Invalid_edge_preprocessing::Invalid_edge_preprocessing (LH_model& lh_model) : Preprocessing (lh_model) {
  if (lh_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing' only applies for metric instances");
}

void Invalid_edge_preprocessing::add () {
  //invalid edges
  list<pair<int, int>> edges = get_invalid_edges_1(lh_model.instance);
  lh_model.nPreprocessings0 = 0;
  for (auto const& [i, j] : edges) {
    auto c1 = lh_model.customersC0Indexes.find(i),
         c2 = lh_model.customersC0Indexes.find(j);
    auto f1 = lh_model.afssF0Indexes.find(i),
         f2 = lh_model.afssF0Indexes.find(j);
    //both are customers
    if (c1 != lh_model.customersC0Indexes.end() && c2 != lh_model.customersC0Indexes.end()) {
      lh_model.model.add(lh_model.x[c1->second][c2->second] == 0);
      ++lh_model.nPreprocessings0;
    }
    //first is afs and second is customer
    if (f1 != lh_model.afssF0Indexes.end() && c2 != lh_model.customersC0Indexes.end()) 
      for (int i = 0; i < lh_model.c0.size(); ++i) {
        lh_model.model.add(lh_model.y[i][f1->second][c2->second] == 0);
        ++lh_model.nPreprocessings0;
      }
    //first is customer and second is afs 
    if (c1 != lh_model.customersC0Indexes.end() && f2 != lh_model.afssF0Indexes.end()) 
      for (int i = 0; i < lh_model.c0.size(); ++i) {
        lh_model.model.add(lh_model.y[c1->second][f2->second][i] == 0);
        ++lh_model.nPreprocessings0;
      }
  }
}
