#include <iostream> 
#include <iterator> 

#include "models/gvrp_models/local_searchs/merge.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/local_search_strategy_enum.hpp"

using namespace std;

using namespace models;
using namespace models::gvrp_models;
using namespace models::gvrp_models::local_searchs;

Merge::Merge (const Gvrp_instance& gvrp_instance, const Gvrp_solution& gvrp_solution, Local_search_strategy_enum strategy_) : Gvrp_local_search(gvrp_instance, gvrp_solution) {
  strategy = strategy_;
}

Merge::~Merge () {
}

Gvrp_solution Merge::run () {
  newSolution = new Gvrp_solution (solution.routes, instance);
  list<list<Vertex>>& routes = newSolution->routes;
  set<int> afss;
  double r1Time,
         r2Time,
         r1r2Time;
  pair<list<list<Vertex>>::iterator, list<list<Vertex>>::iterator> bestRoutes;
  list<Vertex>::iterator r1Vertex;
  int bestReduce = 0;
  bool improvement_found = false;
  //create afss sets
  for (const Vertex& afs : instance.afss)
    afss.insert(afs.id);
  for (list<list<Vertex>>::iterator r1 = routes.begin(); r1 != routes.end(); ++r1) {
    r1Time = r1->begin()->serviceTime;
    for (list<Vertex>::iterator c = next(r1->begin()); c != r1->end(); ++c) 
      r1Time += c->serviceTime + instance.time(c->id, prev(c)->id);
    for (list<list<Vertex>>::iterator r2 = next(r1); r2 != routes.end(); ++r2) {
      if (r1 != r2) {
        list<Vertex>::iterator firstVertexRoute2 = next(r2->begin()),
          lastVertexRoute2 = prev(prev(r2->end()));
        //calculate r2 time
        r2Time = firstVertexRoute2->serviceTime;
        for (list<Vertex>::iterator c = next(firstVertexRoute2); c != r2->end(); r2Time += instance.time(prev(c)->id, c->id) + c->serviceTime, ++c);
        //pruning
        if (r1Time + r2Time >= instance.timeLimit)
          continue;
        //get total time
        r1r2Time = r1Time + instance.time(r2->begin()->id, firstVertexRoute2->id) + r2Time + instance.time(lastVertexRoute2->id, r2->rbegin()->id);
        //try
        for (list<Vertex>::iterator c = r1->begin(); c != --r1->end(); ++c) {
          list<Vertex>::iterator curr = c,
            nex = next(c);
          double spentTime = instance.time(curr->id, firstVertexRoute2->id) - instance.time(curr->id, nex->id) + r2Time + instance.time(lastVertexRoute2->id, nex->id) + r1Time;
          //checking time
          if (spentTime < r1r2Time && r1r2Time - spentTime > bestReduce) {
            //checking fuel 
            double remainingFuel = instance.vehicleFuelCapacity,
                   totalTime = 0.0;
            bool feasible = true;
            list<Vertex> newRoute;
            newRoute.insert(newRoute.end(), r1->begin(), nex);
            newRoute.insert(newRoute.end(), firstVertexRoute2, --r2->end());
            newRoute.insert(newRoute.end(), nex, r1->end());
            for (list<Vertex>::iterator n = next(newRoute.begin()); n != newRoute.end(); ++n) {
              remainingFuel -= instance.fuel(prev(n)->id, n->id);
              totalTime += prev(n)->serviceTime + instance.time(prev(n)->id, n->id);
              if (remainingFuel < 0 || totalTime > instance.timeLimit) {
                feasible = false;
                break;
              }
              if (afss.count(n->id))
                remainingFuel = instance.vehicleFuelCapacity;
            }
            if (!feasible)
              continue;
            improvement_found = true;
            bestReduce = r1r2Time - spentTime;
            bestRoutes = {r1, r2};
            r1Vertex = nex;
            if (strategy == FIRST_IMPROVEMENT) {
              r1 = --routes.end();
              break;
            }
          }
        }
      }
    }
  }
  if (improvement_found) {
    bestRoutes.first->insert(r1Vertex, next(bestRoutes.second->begin()), prev(bestRoutes.second->end()));
    routes.erase(bestRoutes.second);
  }
  return *newSolution;
}
