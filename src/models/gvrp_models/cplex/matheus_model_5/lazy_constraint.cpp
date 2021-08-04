#include "models/gvrp_models/cplex/matheus_model_5/matheus_model_5.hpp"
#include "models/gvrp_models/cplex/matheus_model_5/lazy_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::matheus_model_5;

Lazy_constraint::Lazy_constraint (Matheus_model_5& matheus_model_5_): LazyConstraintCallbackI (matheus_model_5_.env), matheus_model_5(matheus_model_5_) { }
