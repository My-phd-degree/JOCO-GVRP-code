#include "models/local_search.hpp"
#include "models/gvrp_models/gvrp_local_search.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"

Gvrp_local_search::Gvrp_local_search (const Gvrp_instance& gvrp_instance, const Gvrp_solution& gvrp_solution) : Local_search (gvrp_instance, gvrp_solution) {}

Gvrp_local_search::~Gvrp_local_search() {
}

