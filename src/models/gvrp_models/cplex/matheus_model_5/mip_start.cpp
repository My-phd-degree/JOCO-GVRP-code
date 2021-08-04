#include "models/vertex.hpp"
#include "models/gvrp_models/cplex/matheus_model_5/matheus_model_5.hpp"
#include "models/gvrp_models/cplex/matheus_model_5/mip_start.hpp"

#include <iostream>
#include <string>
#include <ilcplex/ilocplex.h>

ILOSTLBEGIN
using namespace std;
using namespace models;
using namespace models::gvrp_models::cplex::matheus_model_5;

Mip_start::Mip_start (const Gvrp_instance& instance, unsigned int time_limit, const Gvrp_solution& gvrp_solution_) : Matheus_model_5 (instance, time_limit), gvrp_solution (gvrp_solution_) {
}

void Mip_start::extraStepsAfterModelCreation () {
}

