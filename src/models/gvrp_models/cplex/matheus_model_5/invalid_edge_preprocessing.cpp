#include "models/gvrp_models/cplex/matheus_model_5/matheus_model_5.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/matheus_model_5/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model_5/invalid_edge_preprocessing.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::matheus_model_5;

Invalid_edge_preprocessing::Invalid_edge_preprocessing (Matheus_model_5& matheus_model_5) : Preprocessing (matheus_model_5) {
  if (matheus_model_5.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing::add () {
  //invalid edges
  list<pair<int, int>> edges = get_invalid_edges_1(matheus_model_5.instance);
  matheus_model_5.nPreprocessings1 = 0;
  for (auto const& [i, j] : edges) {
    auto c1 = matheus_model_5.customersC0Indexes.find(i),
         c2 = matheus_model_5.customersC0Indexes.find(j);
    auto f1 = matheus_model_5.afss_FIndexes.find(i),
         f2 = matheus_model_5.afss_FIndexes.find(j);
    //both are customers
    if (c1 != matheus_model_5.customersC0Indexes.end() && c2 != matheus_model_5.customersC0Indexes.end()) {
      matheus_model_5.model.add(matheus_model_5.x[c1->second][c2->second] == 0);
      ++matheus_model_5.nPreprocessings1;
    }
    //first is afs and second is customer
    if (f1 != matheus_model_5.afss_FIndexes.end() && c2 != matheus_model_5.customersC0Indexes.end()) {
      for (int i = 0; i < matheus_model_5.c0.size(); ++i)
        for (int f = 0; f < matheus_model_5._f.size(); ++f) {
          matheus_model_5.model.add(matheus_model_5.y[i][f][f1->second][c2->second] == 0);
          ++matheus_model_5.nPreprocessings1;
        }
    }
    //first is customer and second is afs 
    if (c1 != matheus_model_5.customersC0Indexes.end() && f2 != matheus_model_5.afss_FIndexes.end()) {
      for (int i = 0; i < matheus_model_5.c0.size(); ++i)
        for (int f = 0; f < matheus_model_5._f.size(); ++f) {
          matheus_model_5.model.add(matheus_model_5.y[c1->second][f2->second][f][i] == 0);
          ++matheus_model_5.nPreprocessings1;
        }
    }
  }
}
