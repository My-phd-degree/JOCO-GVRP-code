#include "models/vertex.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/lh_model/lh_model.hpp"
#include "models/gvrp_models/cplex/lh_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/lh_model/invalid_edge_preprocessing_C7.hpp"
#include "utils/util.hpp"

#include <list>
#include <float.h>

using namespace std;
using namespace utils;
using namespace models;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::lh_model;

Invalid_edge_preprocessing_C7::Invalid_edge_preprocessing_C7 (LH_model& lh_model) : Preprocessing (lh_model) {
  if (lh_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing C7' only applies for metric instances");
}

void Invalid_edge_preprocessing_C7::add () {
  double minFuelI, minFuelJ;
  for (const Vertex& i : lh_model.instance.customers) {
    //min fuel I
    minFuelI = DBL_MAX;
    for (const Vertex& f : lh_model.instance.afss) 
      minFuelI = min(minFuelI, lh_model.instance.fuel(f.id, i.id));
    minFuelI = min(minFuelI, lh_model.instance.fuel(lh_model.instance.depot.id, i.id));
    for (const Vertex& j : lh_model.instance.customers) {
      //min fuel J
      minFuelJ = DBL_MAX;
      for (const Vertex& f : lh_model.instance.afss) 
        minFuelJ = min(minFuelJ, lh_model.instance.fuel(j.id, f.id));
      minFuelJ = min(minFuelJ, lh_model.instance.fuel(j.id, lh_model.instance.depot.id));
      for (const Vertex& f : lh_model.instance.afss) 
        if (minFuelI + lh_model.instance.fuel(i.id, f.id) > lh_model.instance.vehicleFuelCapacity 
            || lh_model.instance.fuel(f.id, j.id) + minFuelJ > lh_model.instance.vehicleFuelCapacity 
            || lh_model.instance.time(lh_model.instance.depot.id, i.id) + lh_model.instance.time(i.id, f.id) + i.serviceTime + lh_model.instance.time(f.id, j.id) + f.serviceTime + lh_model.instance.time(j.id, lh_model.instance.depot.id) + j.serviceTime > lh_model.instance.timeLimit) { 
          int fIndex = lh_model.afssF0Indexes[f.id];
          lh_model.model.add(lh_model.y[lh_model.customersC0Indexes[i.id]][fIndex][lh_model.customersC0Indexes[j.id]] == 0);
          ++lh_model.nPreprocessings3;
        }
    }
  }
}
