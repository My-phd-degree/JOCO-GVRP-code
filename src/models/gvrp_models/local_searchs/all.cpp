#include <iostream> 
#include <iterator> 

#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/local_search_strategy_enum.hpp"
#include "models/gvrp_models/local_searchs/merge.hpp"
#include "models/gvrp_models/local_searchs/all.hpp"
#include "models/gvrp_models/local_searchs/fsRemove.hpp"
#include "models/gvrp_models/local_searchs/swap.hpp"

using namespace std;

using namespace models;
using namespace models::gvrp_models;
using namespace models::gvrp_models::local_searchs;

All::All (const Gvrp_instance& gvrp_instance, const Gvrp_solution& gvrp_solution, Local_search_strategy_enum strategy_) : Gvrp_local_search(gvrp_instance, gvrp_solution) {
  strategy = strategy_;
}

All::~All(){}

Gvrp_solution All::run () {
  newSolution = new Gvrp_solution (solution.routes, instance);
  double newCost = solution.calculateCost(),
         oldCost;
  while (true){
    Gvrp_solution currSolution = *newSolution;
    oldCost = newCost;
    //merge
    Merge merge (instance, currSolution, strategy);
    currSolution = merge.run();
    //swap
    /*
    Swap swap (instance, currSolution, strategy);
    currSolution = swap.run();
    cout<<"Swap: "<<endl;
    cout<<currSolution<<endl;
    */
    //fs remove
    FsRemove fsRemove (instance, currSolution);
    currSolution = fsRemove.run();
    //get cost
    newCost = currSolution.calculateCost();
    if (oldCost > newCost) {
      delete newSolution;
      newSolution = new Gvrp_solution (currSolution);
    } else 
      break;
  }
  return *newSolution;
}
