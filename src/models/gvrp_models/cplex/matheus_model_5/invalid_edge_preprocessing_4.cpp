#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/matheus_model_5/matheus_model_5.hpp"
#include "models/gvrp_models/cplex/matheus_model_5/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model_5/invalid_edge_preprocessing_4.hpp"
#include "utils/util.hpp"

#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::matheus_model_5;

Invalid_edge_preprocessing_4::Invalid_edge_preprocessing_4 (Matheus_model_5& matheus_model_5) : Preprocessing (matheus_model_5) {
  if (matheus_model_5.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 4' only applies for metric instances");
}

void Invalid_edge_preprocessing_4::add () {
  for (int i = 0; i < matheus_model_5.c0.size(); ++i) {
    const Vertex * vertexI = matheus_model_5.c0[i];
    for (int j = 0; j < matheus_model_5.c0.size(); ++j) {
      const Vertex * vertexJ = matheus_model_5.c0[j];
      for (int f_ = 0; f_ < matheus_model_5._f.size(); ++f_) {
        for (int r_ = 0; r_ < matheus_model_5._f.size(); ++r_) {
          bool valid = false;
          for (int f = 0; f < matheus_model_5.gvrp_afs_tree->f0.size(); ++f) {
            const Vertex * vertexF = matheus_model_5.gvrp_afs_tree->f0[f];
            for (int r = 0; r < matheus_model_5.gvrp_afs_tree->f0.size(); ++r) {
              const Vertex * vertexR = matheus_model_5.gvrp_afs_tree->f0[r];
              if (matheus_model_5.instance.fuel(vertexF->id, vertexI->id) + matheus_model_5.customerToAfsFuel(i, f_) <= matheus_model_5.instance.vehicleFuelCapacity 
                  && matheus_model_5.afsToCustomerFuel(r_, j) + matheus_model_5.instance.fuel(vertexJ->id, vertexR->id) <= matheus_model_5.instance.vehicleFuelCapacity) {
                //get f_, and r_ indexes
                int f__, r__;
                for (int k = 0; k < matheus_model_5.gvrp_afs_tree->f0.size(); ++k) {
                  if (matheus_model_5.gvrp_afs_tree->f0[k]->id == vertexF->id)
                    f__ = k;
                  if (matheus_model_5.gvrp_afs_tree->f0[k]->id == vertexR->id)
                    r__ = k;
                }
                //check time
                if (matheus_model_5.gvrp_afs_tree->times[f__] + matheus_model_5.instance.time(vertexF->id, vertexI->id) + matheus_model_5.time(i, f_, r_, j) + vertexJ->serviceTime + matheus_model_5.instance.time(vertexJ->id, vertexR->id) + matheus_model_5.gvrp_afs_tree->times[r__] <= matheus_model_5.instance.timeLimit) { 
                  valid = true;
                  f = matheus_model_5.gvrp_afs_tree->f0.size();
                  break;
                }
              }
            }
          }
          if (!valid) {
            matheus_model_5.model.add(matheus_model_5.y[i][f_][r_][j] == 0);
            ++matheus_model_5.nPreprocessings4;
          } else {
            const Vertex * vertexF_ = matheus_model_5._f[f_],
                        * vertexR_ = matheus_model_5._f[r_];
            for (int f = 0; f < matheus_model_5._f.size(); ++f) {
              const Vertex * vertexF = matheus_model_5._f[f];
              for (int r = 0; r < matheus_model_5._f.size(); ++r) {
                const Vertex * vertexR = matheus_model_5._f[r];
                if ((f_ != f || r_ != r) && 
                    matheus_model_5.customerToAfsFuel(i, f) <= matheus_model_5.customerToAfsFuel(i, f_) &&
                    matheus_model_5.afsToCustomerFuel(r, j) <= matheus_model_5.afsToCustomerFuel(r_, j) &&
                    matheus_model_5.instance.time(vertexI->id, vertexF->id) <= matheus_model_5.instance.time(vertexI->id, vertexF_->id) &&
                    matheus_model_5.instance.time(vertexR->id, vertexJ->id) <= matheus_model_5.instance.time(vertexR_->id, vertexJ->id) &&
                    matheus_model_5.instance.distances[vertexI->id][vertexF->id] <= matheus_model_5.instance.distances[vertexI->id][vertexF_->id] &&
                    matheus_model_5.instance.distances[vertexR->id][vertexJ->id] <= matheus_model_5.instance.distances[vertexR_->id][vertexJ->id]) {
                  //get f, and r indexes
                  int f1, r1, f2, r2;
                  for (int k = 0; k < matheus_model_5.gvrp_afs_tree->f0.size(); ++k) {
                    if (matheus_model_5.gvrp_afs_tree->f0[k]->id == vertexF_->id)
                      f1 = k;
                    if (matheus_model_5.gvrp_afs_tree->f0[k]->id == vertexR_->id)
                      r1 = k;
                    if (matheus_model_5.gvrp_afs_tree->f0[k]->id == vertexF->id)
                      f2 = k;
                    if (matheus_model_5.gvrp_afs_tree->f0[k]->id == vertexR->id)
                      r2 = k;
                  }
                  //check time
                  if (matheus_model_5.gvrp_afs_tree->pairTimes[f2][r2] <= matheus_model_5.gvrp_afs_tree->pairTimes[f1][r1] &&
                      matheus_model_5.gvrp_afs_tree->pairCosts[f2][r2] <= matheus_model_5.gvrp_afs_tree->pairCosts[f1][r1]) { 
                    ++matheus_model_5.nPreprocessings4;
                    matheus_model_5.model.add(matheus_model_5.y[i][f_][r_][j] == 0);
                    f = matheus_model_5._f.size();
                    break;
                  }
                }
              }
            }
          }
        }
      }
    }
  }
}
