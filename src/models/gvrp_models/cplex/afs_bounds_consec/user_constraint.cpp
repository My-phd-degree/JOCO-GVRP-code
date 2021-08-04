#include "models/gvrp_models/cplex/afs_bounds_consec/afs_bounds_consec.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/user_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::afs_bounds_consec;

User_constraint::User_constraint (Afs_bounds_consec& afs_bounds_consec_): UserCutCallbackI (afs_bounds_consec_.env), afs_bounds_consec(afs_bounds_consec_), EPS(1e-3) { }
