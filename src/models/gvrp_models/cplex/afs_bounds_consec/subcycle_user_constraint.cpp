#include "models/vertex.hpp"
#include "models/dsu.hpp"
#include "models/cplex/mip_depth.hpp"
#include "models/gvrp_models/cplex/gvrp_lp_kk_heuristic.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/afs_bounds_consec.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/lazy_constraint.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/subcycle_user_constraint.hpp"
#include "models/bpp_models/bpp_instance.hpp"
#include "models/bpp_models/cplex/bpp_model.hpp"
#include "utils/util.hpp"

#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <list>
#include <ilcplex/ilocplex.h>
#include <lemon/gomory_hu.h>
#include <lemon/concepts/graph.h>
#include <lemon/list_graph.h>
#include <iostream>

using namespace std;
using namespace utils;
using namespace models;
using namespace models::cplex;
using namespace models::bpp_models;
using namespace models::bpp_models::cplex;
using namespace models::gvrp_models::cplex;
using namespace models::gvrp_models::cplex::afs_bounds_consec;
using namespace lemon;
using namespace lemon::concepts;

Subcycle_user_constraint::Subcycle_user_constraint (Afs_bounds_consec& afs_bounds_consec_) : User_constraint (afs_bounds_consec_) {}

IloCplex::CallbackI* Subcycle_user_constraint::duplicateCallback() const {
  return new(getEnv()) Subcycle_user_constraint (*this);
}

void Subcycle_user_constraint::main() {
  //node callback
  Depth const* const d = (Depth *) getNodeData();
  IloInt depth = d ? d->depth : 0;
  if (depth > 0) {
//    abortCutLoop();
//    return;
  }
  //setup
  const int sc0 = afs_bounds_consec.c0.size(),
        s_f = afs_bounds_consec._f.size();
  int bppNRoutesLB, 
      improvedMSTNRoutesLB, 
      maxNRoutes;
  DSU dsu (sc0);
  IloEnv env = getEnv();
  IloExpr lhs (env);
  vector<ListGraph::Node> nodes (sc0);
  ListGraph graph;
  unordered_multimap<int, int> subcomponents (sc0);
  unordered_set<int> component;
  vector<bool> visited (sc0, false);
  list<unordered_set<int>> components;
  queue<int> q;
  //get values
  Matrix2DVal x_vals (env, sc0);
  Matrix4DVal y_vals (env, sc0);
  for (int i = 0; i < sc0; ++i) {
    x_vals[i] = IloNumArray (env, sc0, 0, 1, IloNumVar::Float);
    y_vals[i] = Matrix3DVal (env, s_f);
    getValues(x_vals[i], afs_bounds_consec.x[i]);
    for (int f = 0; f < s_f; ++f) {
      y_vals[i][f] = Matrix2DVal (env, s_f);
      for (int r = 0; r < s_f; ++r) {
        y_vals[i][f][r] = IloNumArray (env, sc0, 0, 1, IloNumVar::Float);
        getValues(y_vals[i][f][r], afs_bounds_consec.y[i][f][r]);
      }
    }
  }
  //creating nodes
  for (int i = 0; i < sc0; ++i) 
    nodes[i] = graph.addNode();
  //get components
  for (int i = 1; i < sc0; ++i) {
    if (visited[i])
      continue;
    ListGraph::EdgeMap<double> weight(graph); 
    //bfs
    component.insert(i);
    q.push(i);
    while (!q.empty()) {
      int curr = q.front();
      visited[curr] = true;
      q.pop();
      for (int j = 0; j < sc0; ++j) {
        double currToJFlow = 0.0, 
               jToCurrFlow = 0.0;
        bool flowExists = false;
        if (x_vals[curr][j] > EPS) {
          currToJFlow = x_vals[curr][j];
          flowExists = true;
          x_vals[curr][j] = 0;
        }
        if (x_vals[j][curr] > EPS) {
          jToCurrFlow = x_vals[j][curr];
          flowExists = true;
          x_vals[j][curr] = 0;
        }
        for (int f = 0; f < s_f; ++f) 
          for (int r = 0; r < s_f; ++r) {
            if (y_vals[curr][f][r][j] > EPS) {
              currToJFlow += y_vals[curr][f][r][j];
              flowExists = true;
              y_vals[curr][f][r][j] = 0;
            }
            if (y_vals[j][f][r][curr] > EPS) {
              jToCurrFlow += y_vals[j][f][r][curr];
              flowExists = true;
              y_vals[j][f][r][curr] = 0;
            }
          }
        if (flowExists) {
          if (!visited[j]) {
            component.insert(j);
            q.push(j);
          }
          nodes[curr];
          nodes[j];
          auto e = graph.addEdge(nodes[curr], nodes[j]);
          weight[e] = currToJFlow;
          e = graph.addEdge(nodes[j], nodes[curr]);
          weight[e] = jToCurrFlow;
        }
      } 
    }
    //gh
    GomoryHu<ListGraph, ListGraph::EdgeMap<double> > gh (graph, weight);
    gh.run();
    //get subcycles
    for (int i : component)
      for (int j : component)
        if (gh.minCutValue(nodes[j], nodes[j]) >= 2.0) 
          dsu.join(i, j);
        else
          cout<<"ooooopssss "<<i<<" and "<<j<<endl;
    component.clear();
  }
  //get subcomponents
  for (int i = 0; i < sc0; ++i) 
    subcomponents.insert(make_pair(dsu.findSet(i), i));
  //store components
  unordered_multimap<int, int>::iterator it = subcomponents.begin();
  int j = it->first;
  for (; it != subcomponents.end(); it++) {
    if (it->first != j) {
      j = it->first;
      components.push_back(component);
      component.clear();
    } 
    component.insert(it->second);
  }
  components.push_back(component);
  //end of multimap 
  //inequallitites
  for (const unordered_set<int>& S : components) 
    if (!S.count(0)) {
      //\sum_{v_i \in V'\S} \sum_{v_j \in S} x_{ij} + \sum_{v_f \in F_0} y_{ifj} \geqslant 1 
      //lhs
      for (int i = 0; i < sc0; ++i) 
        if (!S.count(i)) 
          for (int j : S) {
            lhs += afs_bounds_consec.x[i][j];
            for (int f = 0; f < s_f; ++f)
              for (int r = 0; r < s_f; ++r)
                lhs += afs_bounds_consec.y[i][f][r][j];
          }
      //rhs
      const int sS = S.size();
      vector<const Vertex *> vertices (sS + 1);
      int j = 0;
      for (int i : S) {
        vertices[j] = afs_bounds_consec.c0[i];
        ++j;
      }
      vertices[j] = afs_bounds_consec.c0[0];
      //get n routes lbs
      const auto& closestsTimes = calculateClosestsGVRPCustomers(afs_bounds_consec.gvrpReducedGraphTimes, vertices);
      //get mst
      improvedMSTNRoutesLB = int(ceil(calculateGvrpLBByImprovedMSTTime(vertices, closestsTimes, afs_bounds_consec.gvrpReducedGraphTimes)/afs_bounds_consec.instance.timeLimit));
      //bin packing
      bppNRoutesLB = calculateGVRP_BPP_NRoutesLB(afs_bounds_consec.instance, vertices, closestsTimes, afs_bounds_consec.BPPTimeLimit);
      maxNRoutes = max(improvedMSTNRoutesLB, bppNRoutesLB);
      if (improvedMSTNRoutesLB == maxNRoutes) 
        ++afs_bounds_consec.nImprovedMSTNRoutesLB;
      if (bppNRoutesLB == maxNRoutes) 
        ++afs_bounds_consec.nBPPNRoutesLB;
      lhs -= maxNRoutes;
      try {
        add(lhs >= 0).end();
      } catch(IloException& e) {
        cerr << "Exception while adding user constraint" << e.getMessage() << "\n";
        throw;
      }
      lhs.end();
      lhs = IloExpr(env);
    }
  //clean
  for (int i = 0; i < sc0; ++i) {
    x_vals[i].end();
    for (int f = 0; f < s_f; ++f) {
      for (int r = 0; r < s_f; ++r) 
      y_vals[i][f][r].end();
      y_vals[i][f].end();
    }
    y_vals[i].end();
  }
  x_vals.end();
  y_vals.end();
}
