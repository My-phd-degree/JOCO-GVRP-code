#include "models/vertex.hpp"
#include "models/gvrp_models/cplex/lh_model/lh_model.hpp"
#include "models/gvrp_models/cplex/lh_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/lh_model/invalid_edge_preprocessing_C6.hpp"
#include "utils/util.hpp"

#include <iostream>
#include <list>
#include <string>
#include <float.h>

using namespace std;
using namespace utils;
using namespace models;
using namespace models::gvrp_models::cplex::lh_model;

Invalid_edge_preprocessing_C6::Invalid_edge_preprocessing_C6 (LH_model& lh_model) : Preprocessing (lh_model) {
  if (lh_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing C6' only applies for metric instances");
}

void Invalid_edge_preprocessing_C6::add () {
  for (const Vertex& i : lh_model.instance.customers) 
    for (const Vertex& j : lh_model.instance.customers) {
      //check if edge (i, j) can be feasible
      double minFuelI = DBL_MAX, 
             minFuelJ = DBL_MAX;
      for (const Vertex& f : lh_model.instance.afss) {
        minFuelI = min(minFuelI, lh_model.instance.fuel(f.id, i.id));
        minFuelJ = min(minFuelJ, lh_model.instance.fuel(j.id, f.id));
      }
      minFuelI = min(minFuelI, lh_model.instance.fuel(lh_model.instance.depot.id, i.id));
      minFuelJ = min(minFuelJ, lh_model.instance.fuel(j.id, lh_model.instance.depot.id));
      if (minFuelI + lh_model.instance.fuel(i.id, j.id) + minFuelJ > lh_model.instance.vehicleFuelCapacity || 
          lh_model.instance.time(lh_model.instance.depot.id, i.id) + i.serviceTime + lh_model.instance.time(i.id, j.id) + j.serviceTime + lh_model.instance.time(j.id, lh_model.instance.depot.id) > lh_model.instance.timeLimit) {
        lh_model.model.add(lh_model.x[lh_model.customersC0Indexes[i.id]][lh_model.customersC0Indexes[j.id]] == 0);
        ++lh_model.nPreprocessings2;
      }
    }
}
