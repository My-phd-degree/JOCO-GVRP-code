#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"

using namespace models::gvrp_models::cplex::cubic_model;

Extra_constraint::Extra_constraint (Cubic_model& cubic_model_) : cubic_model (cubic_model_) {
}

Extra_constraint::~Extra_constraint () {}
