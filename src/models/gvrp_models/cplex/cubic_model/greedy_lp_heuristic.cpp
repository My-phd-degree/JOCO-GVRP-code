#include "models/vertex.hpp"
#include "models/cplex/mip_depth.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/greedy_lp_heuristic.hpp"
#include "models/gvrp_models/cplex/gvrp_lp_kk_heuristic.hpp"
#include "models/gvrp_models/local_searchs/fsRemove.hpp"

#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <cmath>
#include <list>
#include <ilcplex/ilocplex.h>
#include <iostream>

ILOSTLBEGIN

using namespace std;
using namespace models;
using namespace models::cplex;
using namespace models::gvrp_models;
using namespace models::gvrp_models::local_searchs;
using namespace models::gvrp_models::cplex;
using namespace models::gvrp_models::cplex::cubic_model;

Greedy_lp_heuristic::Greedy_lp_heuristic (Cubic_model& cubic_model) : Heuristic_callback (cubic_model) {}

IloCplex::CallbackI* Greedy_lp_heuristic::duplicateCallback() const {
  return new(getEnv()) Greedy_lp_heuristic (*this);
}

void Greedy_lp_heuristic::main() {
  try {
    //node callback
    Depth const* const d = (Depth *) getNodeData();
    IloInt depth = d ? d->depth : 0;
    if (depth > 10 && depth%10 == 0) 
      return;
    //setup
    int depot = cubic_model.instance.depot.id;
    const int sall = cubic_model.all.size();
    IloEnv env = getEnv();
    IloExpr lhs (env);
    list<list<Vertex>> routes;
    list<Vertex> route;
    unordered_set<int> customers (sall);
    bool valid = true,
         frac_sol = false;
    double maxVal,
           remainingFuel,
           remainingTime,
           cost = 0;
    int currNode,
        nextNode;
    cubic_model.x_vals = Matrix3DVal (env, cubic_model.instance.maxRoutes);
    //get values
    for (int k = 0; k < cubic_model.instance.maxRoutes; ++k) {
      cubic_model.x_vals[k] = Matrix2DVal (env, sall);
      for (int i = 0; i < sall; ++i) {
        cubic_model.x_vals[k][i] = IloNumArray (env, sall, 0, 1, IloNumVar::Float);
        getValues(cubic_model.x_vals[k][i], cubic_model.x[k][i]);
        //check x values
        for (int j = 0; j < sall; ++j)
          if (abs(round(cubic_model.x_vals[k][i][j]) - cubic_model.x_vals[k][i][j]) > cubic_model.INTEGRALITY_TOL) 
            frac_sol = true;
      }
    }
    if (!frac_sol) 
      return;
    //checking the depot neighboring
    for (int k = 0; k < cubic_model.instance.maxRoutes; ++k) {
      //setup
      remainingFuel = cubic_model.instance.vehicleFuelCapacity;
      remainingTime = cubic_model.instance.timeLimit;
      currNode = depot;
      nextNode = depot;
      valid = true;
      //dfs
      do {
        maxVal = -1.0;
        for (int j = 0; j < sall; ++j)
          if (cubic_model.x_vals[k][currNode][j] > maxVal) {
            nextNode = j;
            maxVal = cubic_model.x_vals[k][currNode][j];
          }
        if (maxVal > 0.0 && cubic_model.instance.fuel(currNode, nextNode) <= remainingFuel && remainingTime - cubic_model.instance.time(currNode, nextNode) - cubic_model.all[currNode]->serviceTime >= 0.0 && !customers.count(nextNode)) {
          remainingFuel -= cubic_model.instance.fuel(currNode, nextNode);
          remainingTime -= (cubic_model.instance.time(currNode, nextNode) + cubic_model.all[currNode]->serviceTime);
          route.push_back(Vertex(*cubic_model.all[currNode]));
          currNode = nextNode;
          if (cubic_model.afss.count(currNode))
            remainingFuel = cubic_model.instance.vehicleFuelCapacity;
          else if (currNode != depot)
            customers.insert(currNode);
        } else {
          valid = false;
          break; 
        }
      } while (currNode != depot);
      //insert route in the routes
      if (valid && route.size() > 1) {
        route.push_back(Vertex(*cubic_model.all[currNode]));
        routes.push_back(route);
      }
      route = list<Vertex> ();
    }
    //if valid solution
    if (customers.size() == cubic_model.instance.customers.size()) {
      Gvrp_solution gvrp_solution (routes, cubic_model.instance);
      if (gvrp_solution.getInfeasibilities().size() > 0)
        return;
      FsRemove fsRemove (cubic_model.instance, gvrp_solution);
      gvrp_solution = fsRemove.run();
      routes = gvrp_solution.routes;
      cost = gvrp_solution.calculateCost();
      //better solution found
      if (cost - getIncumbentObjValue() < -1e-6) {
        ++cubic_model.nGreedyLP;
        cout.precision(17);
        cout<<getIncumbentObjValue()<<" to "<<cost<<endl;
        //set new solution
        //reset all the values
        IloNumArray e_vals = IloNumArray (env, sall, 0, 1, IloNumVar::Float);
        cubic_model.x_vals = Matrix3DVal (cubic_model.env, sall);
        //create vals
        for (int k = 0; k < cubic_model.instance.maxRoutes; ++k) {
          cubic_model.x_vals[k] = Matrix2DVal (cubic_model.env, sall);
          for (int i = 0; i < sall; ++i) {
            cubic_model.x_vals[k][i] = IloNumArray (cubic_model.env, sall, 0, 1, IloNumVar::Int);
            for (int j = 0; j < sall; ++j) 
              cubic_model.x_vals[k][i][j] = 0.0;
          }
        }
        //get values
        IloNumVarArray vars (cubic_model.env);
        IloNumArray vals (cubic_model.env);
        double currFuel;
        int k = 0;
        for (const list<Vertex>& route : routes) {
          currFuel = cubic_model.instance.vehicleFuelCapacity;
          list<Vertex>::const_iterator curr = route.begin(), 
            prev = curr;
          for (++curr; curr != route.end(); prev = curr, ++curr) {
            int i = prev->id, 
                j = curr->id;
            cubic_model.x_vals[k][i][j] = 1;
            //is a customer
            if (cubic_model.customers.count(j)) {
              currFuel -= cubic_model.instance.fuel(i, j);
              e_vals[j] = currFuel;
            } else 
              currFuel = cubic_model.instance.vehicleFuelCapacity;
          }
          ++k;
        }



      /*
      Gvrp_solution gvrp_solution (routes, cubic_model.instance);
      cout<<gvrp_solution<<endl;
      cout<<" ";
      for (int i = 0; i < cubic_model.c0.size(); ++i){
        cout<<" ";
        if (i <=9)
          cout<<" ";
        cout<<i;
      }
      cout<<endl;
      for (int i = 0; i < cubic_model.c0.size(); ++i){
        cout<<i<<" ";
        if (i <= 9)
          cout<<" ";
        for (int j = 0; j < cubic_model.c0.size(); ++j) {
          cout<<abs(cubic_model.x_vals[i][j])<<"  ";
        }
        cout<<endl;
      }
      for (int f = 0; f < cubic_model.f0.size(); ++f){
        cout<<"AFS: "<<f<<endl;
        cout<<" ";
        for (int i = 0; i < cubic_model.c0.size(); ++i){
          cout<<" ";
          if (i <=9)
            cout<<" ";
          cout<<i;
        }
        cout<<endl;
        for (int i = 0; i < cubic_model.c0.size(); ++i){
          cout<<i<<" ";
          if (i <= 9)
            cout<<" ";
          for (int j = 0; j < cubic_model.c0.size(); ++j)
            cout<<abs(cubic_model.y_vals[i][f][j])<<"  ";
          cout<<endl;
        }
      }
      for (int i = 0; i < cubic_model.c0.size(); ++i)
        cout<<i<<": "<<cubic_model.c0[i]->id<<endl;
    */






        //set values
        //e
        for (int i : cubic_model.customers) {
          vars.add(cubic_model.e[i]);
          vals.add(e_vals[i]);
        }
        //x
        for (int k = 0; k < cubic_model.instance.maxRoutes; ++k) {
          for (int i = 0; i < sall; ++i) {
            for (int j = 0; j < sall; ++j) {
              vars.add(cubic_model.x[k][i][j]);
              vals.add(cubic_model.x_vals[k][i][j]);
            }
          }
        }
        setSolution(vars, vals, cost);
        //clean e
        e_vals.end();
      }
      //clean vals
      for (int k = 0; k < cubic_model.instance.maxRoutes; ++k) {
        cubic_model.x_vals[k].end();
        for (int i = 0; i < sall; ++i) 
          cubic_model.x_vals[k][i].end();
      }
      cubic_model.x_vals.end();
    } 
  } catch (IloException& e) {
    throw e;
  } catch (string& s) {
    throw s;
  } catch (...) {
    throw string("Error in setting MIP start solution");
  }
}
