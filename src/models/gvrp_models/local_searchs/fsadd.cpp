#include <iostream> 
#include <random>
#include <iterator> 

#include "models/gvrp_models/local_searchs/fsadd.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/local_search_strategy_enum.hpp"

using namespace std;

using namespace models;
using namespace models::gvrp_models;
using namespace models::gvrp_models::local_searchs;

FSAdd::FSAdd (const Gvrp_instance& gvrp_instance, const Gvrp_solution& gvrp_solution, Local_search_strategy_enum strategy_) : Gvrp_local_search(gvrp_instance, gvrp_solution) {
  strategy = strategy_;
}

FSAdd::~FSAdd (){}

Gvrp_solution FSAdd::run () {
  newSolution = new Gvrp_solution (solution.routes, instance);
  /*
  list<list<Vertex>>& routes = newSolution->routes;
  //customers set
  set<int> customers;
  for (const Vertex& customer : instance.customers)
    customers.insert(customer.id);
  //rng
  random_device rand_dev;
  mt19937 generator(rand_dev());
  uniform_int_distribution<int> distr(0, route.size());

  list<list<Vertex>>::iterator route = next(routes.begin(), distr(generator));
  for (list<Vertex>::iterator node = next(next(route.begin())); node != prev(route.end()); ++node)
    if (customers.count(node->id) && customers.count(prev(node)->id))
      
  for (int i = 0; i < 10; ++i)
    cout << distr(generator) << '\n';
    */
  return *newSolution;
}
