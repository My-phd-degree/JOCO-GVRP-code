#include "utils/util.hpp"
#include "models/vertex.hpp"
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/lazy_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/subcycle_lazy_constraint.hpp"

#include <set>
#include <queue>
#include <list>
#include <ilcplex/ilocplex.h>

using namespace std;
using namespace utils;
using namespace models;
using namespace models::gvrp_models::cplex::cubic_model;

Subcycle_lazy_constraint::Subcycle_lazy_constraint (Cubic_model& cubic_model_) : Lazy_constraint (cubic_model_) {}

IloCplex::CallbackI* Subcycle_lazy_constraint::duplicateCallback() const {
  return new(getEnv()) Subcycle_lazy_constraint (*this);
}

void Subcycle_lazy_constraint::main() {
  int depot = cubic_model.instance.depot.id;
  IloEnv env = getEnv();
  IloExpr lhs(env);
  //get values
  Matrix3DVal x_vals (env, cubic_model.instance.maxRoutes);
  for (int k = 0; k < cubic_model.instance.maxRoutes; k++) {
    x_vals[k] = IloArray<IloNumArray> (env, cubic_model.all.size());
    for (const pair<int, const Vertex *>& p : cubic_model.all) {
      int i = p.first;
      x_vals[k][i] = IloNumArray (env, cubic_model.all.size(), 0, 1, IloNumVar::Int);
      getValues(x_vals[k][i], cubic_model.x[k][i]);
    }
  }
  //get subcycles
  for (int k = 0; k < cubic_model.instance.maxRoutes; k++) {
    //bfs to remove all edges connected to the depot
    queue<int> q;
    q.push(depot);
    while (!q.empty()) {
      int curr = q.front();
      q.pop();
      for (const pair<int, const Vertex *>& p : cubic_model.all)
        if (x_vals[k][curr][p.first] > 0){
          x_vals[k][curr][p.first] = 0;
          q.push(p.first);
        }        
    }
    //bfs to remove all edges not connected to the depot
    for (const Vertex& customer : cubic_model.instance.customers) {
      set<int> component, customersComponent;
      //checking for neighboring
      bool hasNeighboring = false;
      for (const pair<int, const Vertex *>& p : cubic_model.all)
        if (x_vals[k][customer.id][p.first] > 0){
          hasNeighboring = true;
          break; 
        }
      //checking if bfs is needed
      if (!hasNeighboring)
        continue;
      q.push(customer.id);
      while (!q.empty()) {
        int curr = q.front();
        q.pop();
        component.insert(curr);
        //if it is a customer
        if (cubic_model.customers.count(curr))
          customersComponent.insert(curr);
        for (const pair<int, const Vertex *>& p : cubic_model.all)
          if (x_vals[k][curr][p.first] > 0){
            x_vals[k][curr][p.first] = 0;
            q.push(p.first);
          }
      }
      int i = 1;
      vector<const Vertex *> vertices (customersComponent.size() + 1);
      vertices[0] = &cubic_model.instance.depot;
      for (int customer : customersComponent) {
        vertices[i] = cubic_model.all[customer];
        ++i;
      }
      //get n routes lbs
      const auto& closestsTimes = calculateClosestsGVRPCustomers(cubic_model.gvrpReducedGraphTimes, vertices);
      //get mst
      int improvedMSTNRoutesLB = int(ceil(calculateGvrpLBByImprovedMSTTime(vertices, closestsTimes, cubic_model.gvrpReducedGraphTimes)/cubic_model.instance.timeLimit));
      //bin packing
      int bppNRoutesLB = calculateGVRP_BPP_NRoutesLB(cubic_model.instance, vertices, closestsTimes, cubic_model.BPPTimeLimit);
      int maxNRoutes = max(improvedMSTNRoutesLB, bppNRoutesLB);
      if (improvedMSTNRoutesLB == maxNRoutes) 
        ++cubic_model.nImprovedMSTNRoutesLBLazy;
      if (bppNRoutesLB == maxNRoutes) 
        ++cubic_model.nBPPNRoutesLBLazy;
      for (int k_ = 0; k_ < cubic_model.instance.maxRoutes; k_++) { 
        for (int customer_ : customersComponent) {
          //getting lhs
          for (const pair<int, const Vertex *>& p2 : cubic_model.all) {
            int a = p2.first;
            if (!component.count(a))
              for (int b : component) 
                lhs += cubic_model.x[k_][a][b];
          }
          //getting rhs
          for (int b : component)
//            lhs -= cubic_model.x[k_][b][customer_] * maxNRoutes;
            lhs -= cubic_model.x[k_][b][customer_] * 1;
          try {
            add(lhs >= 0).end();
          } catch(IloException& e) {
            cerr << "Exception while adding lazy constraint" << e.getMessage() << "\n";
            throw;
          }
          lhs.end();
          lhs = IloExpr(env);
        } 
      }
      for (int k_ = 0; k_ < cubic_model.instance.maxRoutes; k_++) 
        //getting lhs
        for (const pair<int, const Vertex *>& p2 : cubic_model.all) {
          int a = p2.first;
          if (!component.count(a))
            for (int b : component) 
              lhs += cubic_model.x[k_][a][b];
        }
      try {
        lhs -= maxNRoutes;
        add(lhs >= 0).end();
      } catch(IloException& e) {
        cerr << "Exception while adding lazy constraint" << e.getMessage() << "\n";
        throw;
      }
    }
    lhs.end();
    lhs = IloExpr(env);
    for (const pair<int, const Vertex *>& p : cubic_model.all){
      int i = p.first;
      x_vals[k][i].end();
    }
  }
  x_vals.end();
}
