#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/cubic_model/invalid_edge_preprocessing_4.hpp"
#include "utils/util.hpp"

#include <list>

using namespace std;
using namespace utils;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::cubic_model;

Invalid_edge_preprocessing_4::Invalid_edge_preprocessing_4 (Cubic_model& cubic_model) : Preprocessing (cubic_model) {
  if (cubic_model.instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 4' only applies for metric instances");
}

void Invalid_edge_preprocessing_4::add () {
  /*
  for (const pair<int, Vertex *>& p : cubic_model.all) {
    int i = p.first;
    for (const pair<int, Vertex *>& p1 : cubic_model.all) {
      int j = p1.first;
      if (i != j) {
        for (int f : cubic_model.afss) {
          if (i != f && j != f) {
            bool hasBetter = false;
            for (int r : cubic_model.afss) {
              if (i != r && j != r && f != r) {
                    matheus_model_4.customerToAfsFuel(i, f) <= matheus_model_4.customerToAfsFuel(i, f_) &&
                    matheus_model_4.afsToCustomerFuel(r, j) <= matheus_model_4.afsToCustomerFuel(r_, j) &&
                    matheus_model_4.instance.time(vertexI->id, vertexF->id) <= matheus_model_4.instance.time(vertexI->id, vertexF_->id) &&
                    matheus_model_4.instance.time(vertexR->id, vertexJ->id) <= matheus_model_4.instance.time(vertexR_->id, vertexJ->id) &&
                    matheus_model_4.instance.distances[vertexI->id][vertexF->id] <= matheus_model_4.instance.distances[vertexI->id][vertexF_->id] &&
                    matheus_model_4.instance.distances[vertexR->id][vertexJ->id] <= matheus_model_4.instance.distances[vertexR_->id][vertexJ->id]) {
                if (cubic_model.instance.fuel(i, r) + cubic_model.instance.fuel(r, j) <= cubic_model.instance.fuel(i, f) + cubic_model.instance.fuel(f, j)) {
                //check feasibility first ...
                  hasBetter = true;
                
                }
              }
            }
            if (!hasBetter) {
              //add preprocessing
            }
            */
  list<pair<int, int>> edges = get_invalid_edges_4(cubic_model.instance, *cubic_model.gvrp_afs_tree);
  cubic_model.nPreprocessings4 = 0;
  for (const auto& [i, j] : edges)
    for (int k = 0; k < cubic_model.instance.maxRoutes; k++) {
      cubic_model.model.add(cubic_model.x[k][i][j] == 0);
      ++cubic_model.nPreprocessings4;
    }
}
