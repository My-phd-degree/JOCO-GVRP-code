#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/cubic_model/invalid_edge_preprocessing.hpp"
#include "utils/util.hpp"

#include <list>

using namespace models::gvrp_models::cplex::cubic_model;
using namespace utils;

Invalid_edge_preprocessing::Invalid_edge_preprocessing (Cubic_model& cubic_model) : Preprocessing (cubic_model) {}

void Invalid_edge_preprocessing::add () {
  list<pair<int, int>> edges = get_invalid_edges_1(cubic_model.instance);
  cubic_model.nPreprocessings1 = 0;
  for (auto const& [i, j] : edges)
    for (int k = 0; k < cubic_model.instance.maxRoutes; k++) {
      ++cubic_model.nPreprocessings1;
      cubic_model.model.add(cubic_model.x[k][i][j] == 0);
    }
}
