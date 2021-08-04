#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/cubic_model/invalid_edge_preprocessing_2.hpp"
#include "utils/util.hpp"

#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models::cplex::cubic_model;

Invalid_edge_preprocessing_2::Invalid_edge_preprocessing_2 (Cubic_model& cubic_model) : Preprocessing (cubic_model) {
  if (cubic_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
}

void Invalid_edge_preprocessing_2::add () {
  list<pair<int, int>> edges = get_invalid_edges_2(cubic_model.instance, *cubic_model.gvrp_afs_tree);
  cubic_model.nPreprocessings2 = 0;
  for (const auto& [i, j] : edges)
    for (int k = 0; k < cubic_model.instance.maxRoutes; k++) {
      ++cubic_model.nPreprocessings2;
      cubic_model.model.add(cubic_model.x[k][i][j] == 0);
    }
}
