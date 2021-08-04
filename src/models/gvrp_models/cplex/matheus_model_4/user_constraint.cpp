#include "models/gvrp_models/cplex/matheus_model_4/matheus_model_4.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/user_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::matheus_model_4;

User_constraint::User_constraint (Matheus_model_4& matheus_model_4_): UserCutCallbackI (matheus_model_4_.env), matheus_model_4(matheus_model_4_), EPS(1e-3) { }
