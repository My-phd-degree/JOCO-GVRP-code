#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/preprocessing.hpp"

using namespace models::gvrp_models::cplex::cubic_model;

Preprocessing::Preprocessing (Cubic_model& cubic_model_) : cubic_model(cubic_model_) {}

Preprocessing::~Preprocessing () {}
