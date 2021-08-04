#include "models/gvrp_models/cplex/matheus_model_4/matheus_model_4.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/invalid_edge_preprocessing_3.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>
#include <string>

using namespace std;
using namespace utils;
using namespace models::gvrp_models::cplex::matheus_model_4;

Invalid_edge_preprocessing_3::Invalid_edge_preprocessing_3 (Matheus_model_4& matheus_model_4) : Preprocessing (matheus_model_4) {
  if (matheus_model_4.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing_3::add () {
  list<pair<int, int>> edges = get_invalid_edges_3 (matheus_model_4.instance, *matheus_model_4.gvrp_afs_tree);
  matheus_model_4.nPreprocessings3 = edges.size();
  for (const auto& [i, j] : edges) {
    matheus_model_4.model.add(matheus_model_4.x[matheus_model_4.customersC0Indexes[i]][matheus_model_4.customersC0Indexes[j]] == 0);
  }
}
