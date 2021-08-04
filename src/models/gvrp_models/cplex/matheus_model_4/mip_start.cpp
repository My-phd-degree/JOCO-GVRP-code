#include "models/vertex.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/matheus_model_4.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/mip_start.hpp"

#include <iostream>
#include <string>
#include <ilcplex/ilocplex.h>

ILOSTLBEGIN
using namespace std;
using namespace models;
using namespace models::gvrp_models::cplex::matheus_model_4;

Mip_start::Mip_start (const Gvrp_instance& instance, unsigned int time_limit, const Gvrp_solution& gvrp_solution_) : Matheus_model_4 (instance, time_limit), gvrp_solution (gvrp_solution_) {
}

void Mip_start::extraStepsAfterModelCreation () {
}

