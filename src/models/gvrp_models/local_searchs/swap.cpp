#include <iostream> 
#include <iterator> 
#include <cmath> 

#include "models/gvrp_models/local_searchs/swap.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/local_search_strategy_enum.hpp"

using namespace std;

using namespace models;
using namespace models::gvrp_models;
using namespace models::gvrp_models::local_searchs;

Swap::Swap (const Gvrp_instance& gvrp_instance, const Gvrp_solution& gvrp_solution, Local_search_strategy_enum strategy_) : Gvrp_local_search(gvrp_instance, gvrp_solution) {
  strategy = strategy_;
}

Swap::~Swap (){}

Gvrp_solution Swap::run () {
  newSolution = new Gvrp_solution (solution.routes, instance);
  list<list<Vertex>>& routes = newSolution->routes;
  set<int> afss;
  pair<list<Vertex>::iterator, list<Vertex>::iterator> bestCustomers;
  double r1Time, r2Time, r1NewTime, r2NewTime, currentFuel;
  int bestReduce = 0;
  bool improvement_found = false;
  bool feasible;
  //create afss sets
  for (const Vertex& afs : instance.afss)
    afss.insert(afs.id);
  int i = 0;
  for (list<list<Vertex>>::iterator r1 = routes.begin(); r1 != routes.end(); ++r1) {
    cout<<"Route "<<i<<endl;
    ++i;
    //get r1 time
    r1Time = r1->begin()->serviceTime;
    for (list<Vertex>::iterator c = next(r1->begin()); c != r1->end(); ++c) 
      r1Time += c->serviceTime + instance.time(prev(c)->id, c->id);
    //try to improve r1
    for (list<Vertex>::iterator c1 = next(r1->begin()); c1 != --r1->end(); ++c1) {
      cout<<c1->id<<endl;
      for(const Vertex& node : *r1)
        cout<<node.id<<", ";
      cout<<endl;
      for (list<Vertex>::iterator c2 = next(c1); c2 != --r1->end(); ++c2) {
        if (!afss.count(c1->id) && !afss.count(c2->id)) {
//        cout<<"\t"<<c2->id<<endl;
          swap(*c1, *c2);
          //check feasibility
          feasible = true;
          r1NewTime = r1->begin()->serviceTime;
          currentFuel = instance.vehicleFuelCapacity;
          for (list<Vertex>::iterator c = next(r1->begin()); c != r1->end(); ++c) {
            r1NewTime += c->serviceTime + instance.time(prev(c)->id, c->id);
            currentFuel -= instance.fuel(prev(c)->id, c->id);
            if (currentFuel < 0 || r1NewTime > r1Time) {
              feasible = false; 
              break;
            }
            if (afss.count(c->id))
              currentFuel = instance.vehicleFuelCapacity;
          }
          if (feasible && r1Time - r1NewTime > bestReduce) {
            bestReduce = r1Time - r1NewTime;
            bestCustomers = {c1, c2};
            improvement_found = true;
            if (strategy == FIRST_IMPROVEMENT) {
              swap(*c1, *c2);
              c1 = --r1->end();
              r1 = --routes.end();
              break;
            }
          }
          swap(*c1, *c2);
        }
      }
    }
    //try to swap with r2
    for (list<list<Vertex>>::iterator r2 = next(r1); r2 != routes.end(); ++r2) {
      //get r2 time
      r2Time = r2->begin()->serviceTime;
      for (list<Vertex>::iterator c = next(r2->begin()); c != r2->end(); ++c) 
        r2Time += instance.time(prev(c)->id, c->id) + c->serviceTime;
      for (list<Vertex>::iterator c1 = next(r1->begin()); c1 != --r1->end(); ++c1) {
        for (list<Vertex>::iterator c2 = next(r2->begin()); c2 != --r2->end(); ++c2) {
          if (!afss.count(c1->id) && !afss.count(c2->id)) {
            swap(*c1, *c2);
            //check feasibility
            feasible = true;
            //check route 1
            r1NewTime = r1->begin()->serviceTime;
            currentFuel = instance.vehicleFuelCapacity;
            for (list<Vertex>::iterator c = next(r1->begin()); c != r1->end(); ++c) {
              r1NewTime += instance.time(prev(c)->id, c->id) + c->serviceTime;
              currentFuel -= instance.fuel(prev(c)->id, c->id);
              if (currentFuel < 0 || r1NewTime > instance.timeLimit) {
                feasible = false; 
                break;
              }
              if (afss.count(c->id))
                currentFuel = instance.vehicleFuelCapacity;
            }
            if (!feasible) {
              swap(*c1, *c2);
              continue;
            }
            //check route 2
            feasible = true;
            r2NewTime = r2->begin()->serviceTime;
            currentFuel = instance.vehicleFuelCapacity;
            for (list<Vertex>::iterator c = next(r2->begin()); c != r2->end(); ++c) {
              r2NewTime += instance.time(prev(c)->id, c->id) + c->serviceTime;
              currentFuel -= instance.fuel(prev(c)->id, c->id);
              if (currentFuel < 0 || r2NewTime > instance.timeLimit) {
                feasible = false; 
                break;
              }
              if (afss.count(c->id))
                currentFuel = instance.vehicleFuelCapacity;
            }
            //final check
            double sub = r1Time + r2Time - r1NewTime - r2NewTime;
            if (feasible && sub > 0 && floor(1000.0 * sub) > 0 && sub > bestReduce) {
              bestReduce = sub;
              bestCustomers = {c1, c2};
              improvement_found = true;
              if (strategy == FIRST_IMPROVEMENT) {
                swap(*c1, *c2);
                c1 = --r1->end();
                r2 = --routes.end();
                r1 = --routes.end();
                break;
              }
            }
            swap(*c1, *c2);
          }
        }
      }
    }
  }
  if (improvement_found) {
    swap(*bestCustomers.first, *bestCustomers.second);
  }
  return *newSolution;
}
