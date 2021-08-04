#include "models/gvrp_models/cplex/matheus_model_4/matheus_model_4.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/lazy_constraint.hpp"

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::gvrp_models::cplex::matheus_model_4;

Lazy_constraint::Lazy_constraint (Matheus_model_4& matheus_model_4_): LazyConstraintCallbackI (matheus_model_4_.env), matheus_model_4(matheus_model_4_) { }
