#include "utils/util.hpp"
#include "models/dsu.hpp"
#include "models/vertex.hpp"
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/user_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/subcycle_user_constraint.hpp"
#include "models/cplex/mip_depth.hpp"

#include <set>
#include <queue>
#include <list>
#include <ilcplex/ilocplex.h>
#include <lemon/gomory_hu.h>
#include <lemon/concepts/graph.h>
#include <lemon/list_graph.h>

using namespace std;
using namespace utils;
using namespace models;
using namespace models::cplex;
using namespace models::gvrp_models::cplex::cubic_model;
using namespace lemon;
using namespace lemon::concepts;

Subcycle_user_constraint::Subcycle_user_constraint (Cubic_model& cubic_model_) : User_constraint (cubic_model_) {}

IloCplex::CallbackI* Subcycle_user_constraint::duplicateCallback() const {
  return new(getEnv()) Subcycle_user_constraint (*this);
}

void Subcycle_user_constraint::main() {
  Depth const* const d = (Depth *)getNodeData();
  IloInt depth = d ? d->depth : 0;
  if (depth > cubic_model.levelSubcycleCallback) {
    abortCutLoop();
    return;
  }
  const int sall = cubic_model.instance.distances.size();
  IloEnv env = getEnv();
  IloExpr lhs(env);
  unordered_set<int> visited;
  queue<int> q;
  int depot = cubic_model.instance.depot.id;
  int k;
  //get values
  Matrix3DVal x_vals (env, cubic_model.instance.maxRoutes);
  for (k = 0; k < cubic_model.instance.maxRoutes; k++) {
    x_vals[k] = IloArray<IloNumArray> (env, sall);
    for (const pair<int, const Vertex *>& p : cubic_model.all) {
      int i = p.first;
      x_vals[k][i] = IloNumArray (env, sall, 0, 1, IloNumVar::Float);
      getValues(x_vals[k][i], cubic_model.x[k][i]);
    }
  }
  //bfs
  for (const pair<int, const Vertex *>& p : cubic_model.all) {
    int i = p.first;
    bool depotVisited = false;
    if (!visited.count(i) && i != depot) {
      unordered_set<int> component;
      component.insert(i);
      visited.insert(i); 
      q.push(i);
      while (!q.empty()) {
        int curr = q.front();
        q.pop();
        if (curr == depot) 
          depotVisited = true;
        for (const pair<int, const Vertex *>& p1 : cubic_model.all) {
          int j = p1.first;
          if (!visited.count(j)) {
            double weight = 0.0;
            for (k = 0; k < cubic_model.instance.maxRoutes; k++) {
              if (x_vals[k][curr][j] > EPS)
                weight += x_vals[k][curr][j];
              if (x_vals[k][j][curr] > EPS)
                weight += x_vals[k][j][curr];
              if (weight > EPS) {
                component.insert(j);
                visited.insert(j);
                q.push(j);
              }
            }
          }
        }
      }
      if (!depotVisited) {
        //get customers from component S
        list<int> customersComponent;
        for (int customer : cubic_model.customers)
          if (component.count(customer))
            customersComponent.push_back(customer);
        if (customersComponent.size() == 0)
          continue;
        vector<const Vertex *> vertices (customersComponent.size() + 1);
        vertices[0] = &cubic_model.instance.depot;
        int i = 1;
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
        //get max
        int maxNRoutes = max(improvedMSTNRoutesLB, bppNRoutesLB);
        try {
          //in edges
          //getting lhs
          for (const pair<int, const Vertex *>& p1 : cubic_model.all) {
            int a = p1.first;
            if (!component.count(a))
              for (int b : component) 
                for (k = 0; k < cubic_model.instance.maxRoutes; k++) {
                  lhs += cubic_model.x[k][a][b];
                }
          }
          //getting rhs
          lhs -= maxNRoutes;
          add(lhs >= 0.0).end();
          lhs.end();
          lhs = IloExpr (env);
          if (improvedMSTNRoutesLB == maxNRoutes) 
            ++cubic_model.nImprovedMSTNRoutesLB;
          if (bppNRoutesLB == maxNRoutes) 
            ++cubic_model.nBPPNRoutesLB;
        } catch(IloException& e) {
          cerr << "Exception while adding lazy constraint" << e.getMessage() << "\n";
          throw;
        }
      }
    }
  }
  //end vars
  for (k = 0; k < cubic_model.instance.maxRoutes; ++k)
    for (const pair<int, const Vertex *>& p : cubic_model.all){
      int i = p.first;
      x_vals[k][i].end();
    }
  x_vals.end();
}
