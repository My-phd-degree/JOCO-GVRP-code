#include "models/gvrp_models/cplex/matheus_model_5/matheus_model_5.hpp"
#include "models/gvrp_models/cplex/matheus_model_5/user_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::matheus_model_5;

User_constraint::User_constraint (Matheus_model_5& matheus_model_5_): UserCutCallbackI (matheus_model_5_.env), matheus_model_5(matheus_model_5_), EPS(1e-3) { }
