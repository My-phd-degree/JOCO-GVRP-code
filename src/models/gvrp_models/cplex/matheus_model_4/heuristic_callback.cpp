#include "models/vertex.hpp"
#include "models/cplex/mip_depth.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/matheus_model_4.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::matheus_model_4;

Heuristic_callback::Heuristic_callback (Matheus_model_4& matheus_model_4_) : IloCplex::HeuristicCallbackI (matheus_model_4_.env), matheus_model_4(matheus_model_4_), EPS(1e-3) {}
