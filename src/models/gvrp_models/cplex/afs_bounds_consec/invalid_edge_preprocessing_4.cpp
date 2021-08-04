#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/afs_bounds_consec.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/preprocessing.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/invalid_edge_preprocessing_4.hpp"
#include "utils/util.hpp"

#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::afs_bounds_consec;

Invalid_edge_preprocessing_4::Invalid_edge_preprocessing_4 (Afs_bounds_consec& afs_bounds_consec) : Preprocessing (afs_bounds_consec) {
  if (afs_bounds_consec.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 4' only applies for metric instances");
}

void Invalid_edge_preprocessing_4::add () {
  for (int i = 0; i < afs_bounds_consec.c0.size(); ++i) {
    const Vertex * vertexI = afs_bounds_consec.c0[i];
    for (int j = 0; j < afs_bounds_consec.c0.size(); ++j) {
      const Vertex * vertexJ = afs_bounds_consec.c0[j];
      for (int f_ = 0; f_ < afs_bounds_consec._f.size(); ++f_) {
        for (int r_ = 0; r_ < afs_bounds_consec._f.size(); ++r_) {
          bool valid = false;
          for (int f = 0; f < afs_bounds_consec.gvrp_afs_tree->f0.size(); ++f) {
            const Vertex * vertexF = afs_bounds_consec.gvrp_afs_tree->f0[f];
            for (int r = 0; r < afs_bounds_consec.gvrp_afs_tree->f0.size(); ++r) {
              const Vertex * vertexR = afs_bounds_consec.gvrp_afs_tree->f0[r];
              if (afs_bounds_consec.instance.fuel(vertexF->id, vertexI->id) + afs_bounds_consec.customerToAfsFuel(i, f_) <= afs_bounds_consec.instance.vehicleFuelCapacity 
                  && afs_bounds_consec.afsToCustomerFuel(r_, j) + afs_bounds_consec.instance.fuel(vertexJ->id, vertexR->id) <= afs_bounds_consec.instance.vehicleFuelCapacity) {
                //get f_, and r_ indexes
                int f__, r__;
                for (int k = 0; k < afs_bounds_consec.gvrp_afs_tree->f0.size(); ++k) {
                  if (afs_bounds_consec.gvrp_afs_tree->f0[k]->id == vertexF->id)
                    f__ = k;
                  if (afs_bounds_consec.gvrp_afs_tree->f0[k]->id == vertexR->id)
                    r__ = k;
                }
                //check time
                if (afs_bounds_consec.gvrp_afs_tree->times[f__] + afs_bounds_consec.instance.time(vertexF->id, vertexI->id) + afs_bounds_consec.time(i, f_, r_, j) + vertexJ->serviceTime + afs_bounds_consec.instance.time(vertexJ->id, vertexR->id) + afs_bounds_consec.gvrp_afs_tree->times[r__] <= afs_bounds_consec.instance.timeLimit) { 
                  valid = true;
                  f = afs_bounds_consec.gvrp_afs_tree->f0.size();
                  break;
                }
              }
            }
          }
          if (!valid) {
            afs_bounds_consec.model.add(afs_bounds_consec.y[i][f_][r_][j] == 0);
            ++afs_bounds_consec.nPreprocessings4;
          } 
        }
      }
    }
  }
}
