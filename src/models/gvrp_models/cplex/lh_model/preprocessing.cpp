#include "models/gvrp_models/cplex/lh_model/lh_model.hpp"
#include "models/gvrp_models/cplex/lh_model/preprocessing.hpp"

using namespace models::gvrp_models::cplex::lh_model;

Preprocessing::Preprocessing (LH_model& lh_model_) : lh_model(lh_model_) {}

Preprocessing::~Preprocessing () {}
