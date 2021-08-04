#include "models/vertex.hpp"
#include "models/objective_function_enum.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/afs_bounds_consec.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/mip_start.hpp"

#include <iostream>
#include <string>
#include <ilcplex/ilocplex.h>

ILOSTLBEGIN
using namespace std;
using namespace models;
using namespace models::gvrp_models::cplex::afs_bounds_consec;

Mip_start::Mip_start (const Gvrp_instance& instance, unsigned int time_limit, const Vertex& afs, const Gvrp_solution& gvrp_solution_, Objective_function_enum objective_function_enum) : Afs_bounds_consec (instance, time_limit, afs, objective_function_enum), gvrp_solution (gvrp_solution_) {
}

void Mip_start::extraStepsAfterModelCreation () {
}

