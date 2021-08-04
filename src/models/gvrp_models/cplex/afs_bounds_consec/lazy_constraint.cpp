#include "models/gvrp_models/cplex/afs_bounds_consec/afs_bounds_consec.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/lazy_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::afs_bounds_consec;

Lazy_constraint::Lazy_constraint (Afs_bounds_consec& afs_bounds_consec_): LazyConstraintCallbackI (afs_bounds_consec_.env), afs_bounds_consec(afs_bounds_consec_) { }
