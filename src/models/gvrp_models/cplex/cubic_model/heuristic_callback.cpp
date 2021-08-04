#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/greedy_lp_heuristic.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::cubic_model;

Heuristic_callback::Heuristic_callback (Cubic_model& cubic_model_) : IloCplex::HeuristicCallbackI (cubic_model_.env), cubic_model(cubic_model_), EPS(1e-3) {}
