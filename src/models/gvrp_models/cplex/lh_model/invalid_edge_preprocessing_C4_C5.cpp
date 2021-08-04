#include "models/gvrp_models/cplex/lh_model/lh_model.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/lh_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/lh_model/invalid_edge_preprocessing_C4_C5.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::lh_model;

Invalid_edge_preprocessing_C4_C5::Invalid_edge_preprocessing_C4_C5 (LH_model& lh_model) : Preprocessing (lh_model) {
  if (lh_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing C4 C5' only applies for metric instances");
}

void Invalid_edge_preprocessing_C4_C5::add () {
  //invalid edges
  lh_model.nPreprocessings1 = 0;
  for (int i = 1; i < lh_model.c0.size(); ++i) {
    double closestAFS = lh_model.customersFuel(0, i);
    for (int f = 0; f < lh_model.f0.size(); ++f)
      closestAFS = min (closestAFS, lh_model.afsToCustomerFuel(f, i));
    if (closestAFS + lh_model.customersFuel(i, 0) > lh_model.instance.vehicleFuelCapacity) {
      lh_model.model.add(lh_model.x[0][i] == 0);
      lh_model.model.add(lh_model.x[i][0] == 0);
      lh_model.nPreprocessings1 += 2;
    }
  }
}
