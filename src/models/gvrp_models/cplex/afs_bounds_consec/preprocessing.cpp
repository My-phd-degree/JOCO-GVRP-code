#include "models/gvrp_models/cplex/afs_bounds_consec/afs_bounds_consec.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/preprocessing.hpp"

using namespace models::gvrp_models::cplex::afs_bounds_consec;

Preprocessing::Preprocessing (Afs_bounds_consec& afs_bounds_consec_) : afs_bounds_consec(afs_bounds_consec_) {}

Preprocessing::~Preprocessing () {}
