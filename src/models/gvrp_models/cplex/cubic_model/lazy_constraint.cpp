#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/lazy_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::cubic_model;

Lazy_constraint::Lazy_constraint (Cubic_model& cubic_model_): LazyConstraintCallbackI (cubic_model_.env), cubic_model(cubic_model_) { }
