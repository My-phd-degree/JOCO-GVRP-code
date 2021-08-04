#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/user_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::cubic_model;

User_constraint::User_constraint (Cubic_model& cubic_model_): UserCutCallbackI (cubic_model_.env), cubic_model(cubic_model_), EPS(1e-3) { }
