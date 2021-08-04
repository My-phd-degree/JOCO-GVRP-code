#include <iostream> 
#include <iterator> 
#include <unordered_set> 

#include "models/gvrp_models/local_searchs/fsRemove.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/local_search_strategy_enum.hpp"

using namespace std;

using namespace models;
using namespace models::gvrp_models;
using namespace models::gvrp_models::local_searchs;

FsRemove::FsRemove (const Gvrp_instance& gvrp_instance, const Gvrp_solution& gvrp_solution) : Gvrp_local_search(gvrp_instance, gvrp_solution) {}

FsRemove::~FsRemove () {}

Gvrp_solution FsRemove::run () {
  newSolution = new Gvrp_solution (solution.routes, instance);
  //create c0
  unordered_set<int> f0;
  for (const Vertex& afs : instance.afss)
    f0.insert(afs.id);
  f0.insert(instance.depot.id);
  //dp to remove redundant AFSs visits 
  for (list<Vertex>& route : newSolution->routes) {
    //AFSs
    list<Vertex>::const_iterator first = route.begin(), 
      second = route.end(); 
    double firstSliceFuel, 
           consumedFuel = 0.0;
    list<Vertex>::const_iterator curr = route.begin(), 
      previous = curr;
    for (++curr; curr != route.end(); previous = curr, ++curr) {
      auto currIndex = f0.find(curr->id);
      consumedFuel += instance.fuel(previous->id, curr->id);
      //is an AFS or depot 
      if (currIndex != f0.end()) {
        if (second == route.end()) {
          second = curr;
          firstSliceFuel = consumedFuel;
        } else {
          if (consumedFuel <= instance.vehicleFuelCapacity) {
            consumedFuel = consumedFuel - instance.fuel(prev(second)->id, second->id) - instance.fuel(second->id, next(second)->id) + instance.fuel(prev(second)->id, next(second)->id);
            cout<<"between ("<<first->id<<", "<<second->id<<", "<<curr->id<<") "<<consumedFuel<<endl;
            route.erase(second);
            firstSliceFuel = consumedFuel;
          } else {
            first = second;
            consumedFuel -= firstSliceFuel;
          }
          second = curr;
        }
      }
    }
  }
  return *newSolution;
}
