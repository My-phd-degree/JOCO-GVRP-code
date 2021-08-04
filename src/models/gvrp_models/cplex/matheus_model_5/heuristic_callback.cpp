#include "models/vertex.hpp"
#include "models/cplex/mip_depth.hpp"
#include "models/gvrp_models/cplex/matheus_model_5/matheus_model_5.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::matheus_model_5;

Heuristic_callback::Heuristic_callback (Matheus_model_5& matheus_model_5_) : IloCplex::HeuristicCallbackI (matheus_model_5_.env), matheus_model_5(matheus_model_5_), EPS(1e-3) {}
