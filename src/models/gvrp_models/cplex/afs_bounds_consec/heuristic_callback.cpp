#include "models/vertex.hpp"
#include "models/cplex/mip_depth.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/afs_bounds_consec.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::afs_bounds_consec;

Heuristic_callback::Heuristic_callback (Afs_bounds_consec& afs_bounds_consec_) : IloCplex::HeuristicCallbackI (afs_bounds_consec_.env), afs_bounds_consec(afs_bounds_consec_), EPS(1e-3) {}
