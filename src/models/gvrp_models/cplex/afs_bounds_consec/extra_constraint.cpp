#include "models/gvrp_models/cplex/afs_bounds_consec/afs_bounds_consec.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/extra_constraint.hpp"

using namespace models::gvrp_models::cplex::afs_bounds_consec;

Extra_constraint::Extra_constraint (Afs_bounds_consec& afs_bounds_consec_) : afs_bounds_consec (afs_bounds_consec_) {
}

Extra_constraint::~Extra_constraint () {}
