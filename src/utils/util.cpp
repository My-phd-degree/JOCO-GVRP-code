#include "models/vertex.hpp"
#include "models/dsu.hpp"
#include "models/vrp_instance.hpp"
#include "models/distances_enum.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/bpp_models/bpp_instance.hpp"
#include "models/bpp_models/cplex/bpp_model.hpp"
#include "utils/util.hpp"

#include <float.h>
#include <climits>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <sstream>
#include <string>
#include <string.h>
#include <cmath>
#include <map>
#include <set>
#include <queue>
#include <dirent.h>
#include <sys/types.h>
#include <list>
#include "SampleConfig.h"

using namespace models;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::cubic_model;
using namespace models::bpp_models;
using namespace models::bpp_models::cplex;

//gvrp

pair<vector<vector<double>>, vector<vector<double>>> utils::calculateGVRPReducedGraphs (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree) {
  int depotId = gvrp_instance.depot.id;
  const int sc0 = gvrp_instance.customers.size() + 1.0,
              stotal = gvrp_instance.distances.size();
  vector<const Vertex *> c0 (sc0);
  vector<vector<double>> closestTime (stotal, vector<double>(stotal, -1.0)); 
  vector<vector<double>> closestDistance (stotal, vector<double>(stotal, -1.0)); 
  //build c0
  c0[0] = &gvrp_instance.depot;
  int i = 1;
  for (const Vertex& customer : gvrp_instance.customers) {
    c0[i] = &customer;
    ++i;
  }
  //for each pair
  for (int i = 0; i < sc0; ++i) {
    const Vertex * customerI =  c0[i];
    int customerIid = customerI->id;
    for (int j = i + 1; j < sc0; ++j) {
      const Vertex * customerJ =  c0[j];
      int customerJid = customerJ->id;
      if (customerIid != customerJid && 
          gvrp_instance.time(depotId, customerIid) + customerI->serviceTime + gvrp_instance.time(customerIid, customerJid) + customerJ->serviceTime + gvrp_instance.time(customerIid, depotId) <= gvrp_instance.timeLimit) {
        //origin afs
        for (int f = 0; f < gvrp_afs_tree.f0.size(); f++) {
          int fId = gvrp_afs_tree.f0[f]->id;
          double spentTime = gvrp_afs_tree.times[f] + gvrp_instance.time(fId, customerIid) + customerI->serviceTime;
          double spentFuel = gvrp_instance.fuel(fId, customerIid);
          if (spentFuel <= gvrp_instance.vehicleFuelCapacity && 
              spentTime + gvrp_instance.time(customerIid, customerJid) + customerJ->serviceTime + gvrp_instance.time(customerJid, depotId) <= gvrp_instance.timeLimit) {
            //destiny afs
            for (int r = 0; r < gvrp_afs_tree.f0.size(); r++) {
              int rId = gvrp_afs_tree.f0[r]->id;
              if (spentTime + gvrp_instance.time(customerIid, customerJid) + customerJ->serviceTime + gvrp_instance.time(customerJid, rId) + gvrp_afs_tree.times[r] <= gvrp_instance.timeLimit) {
                //direct travel
                if (spentFuel + gvrp_instance.fuel(customerIid, customerJid) + gvrp_instance.fuel(customerJid, rId) <= gvrp_instance.vehicleFuelCapacity) {
                  double time = gvrp_instance.time(customerIid, customerJid);
                  double cost = gvrp_instance.distances[customerIid][customerJid];
                  if (closestTime[customerIid][customerJid] < 0.0 || time < closestTime[customerIid][customerJid]) 
                    closestTime[customerIid][customerJid] = closestTime[customerJid][customerIid] = time;
                  if (closestDistance[customerIid][customerJid] < 0.0 || cost < closestDistance[customerIid][customerJid]) 
                    closestDistance[customerIid][customerJid] = closestDistance[customerJid][customerIid] = cost;
                } else {
                  //intermediate travel
                  for (int f_ = 0; f_ < gvrp_afs_tree.f0.size(); f_++) {
                    int f_Id = gvrp_afs_tree.f0[f_]->id;
                    //fuel and time feasible
                    if (spentFuel + gvrp_instance.fuel(customerIid, f_Id) <= gvrp_instance.vehicleFuelCapacity && 
                        spentTime + gvrp_instance.time(customerIid, f_Id) + gvrp_afs_tree.f0[f_]->serviceTime + gvrp_instance.time(f_Id, customerJid) + customerJ->serviceTime + gvrp_instance.time(customerJid, rId) + gvrp_afs_tree.times[r] <= gvrp_instance.timeLimit) {
                      //intermediate afs destiny
                      for (int r_ = 0; r_ < gvrp_afs_tree.f0.size(); r_++) {
                        int r_Id = gvrp_afs_tree.f0[r_]->id;
                        //time feasible
                        if (gvrp_instance.fuel(r_Id, customerJid) + gvrp_instance.fuel(customerJid, rId) <= gvrp_instance.vehicleFuelCapacity) {
                          double time, cost;
                          if (spentTime + gvrp_instance.time(customerIid, f_Id) + gvrp_afs_tree.pairTimes[f_][r_] + gvrp_instance.time(r_Id, customerJid) + customerJ->serviceTime + gvrp_instance.time(customerJid, rId) + gvrp_afs_tree.times[r] <= gvrp_instance.timeLimit) {
                            time = gvrp_instance.time(customerIid, f_Id) + gvrp_afs_tree.pairTimes[f_][r_] + gvrp_instance.time(r_Id, customerJid);
                            cost = gvrp_instance.distances[customerIid][f_Id] + gvrp_afs_tree.pairCosts[f_][r_] + gvrp_instance.distances[r_Id][customerJid];
                          } else if (spentTime + gvrp_instance.time(customerIid, f_Id) + gvrp_afs_tree.times[f_] <= gvrp_instance.timeLimit &&
                              gvrp_afs_tree.times[r_] + gvrp_instance.time(r_Id, customerJid) + customerJ->serviceTime + gvrp_instance.time(customerJid, rId) + gvrp_afs_tree.times[r] <= gvrp_instance.timeLimit) {
                            time = gvrp_instance.time(customerIid, f_Id) + gvrp_afs_tree.times[f_] + gvrp_afs_tree.times[r_] + gvrp_instance.time(r_Id, customerJid);
                            cost = gvrp_instance.distances[customerIid][f_Id] + gvrp_afs_tree.pairCosts[f_][0] + gvrp_afs_tree.pairCosts[0][r_] + gvrp_instance.distances[r_Id][customerJid];
                          }
                          if (closestTime[customerIid][customerJid] < 0.0 || 
                              time < closestTime[customerIid][customerJid]) 
                            closestTime[customerIid][customerJid] = closestTime[customerJid][customerIid] = time;
                          if (closestDistance[customerIid][customerJid] < 0.0 || 
                              cost < closestDistance[customerIid][customerJid]) 
                            closestDistance[customerIid][customerJid] = closestDistance[customerJid][customerIid] = cost;
                        }
                      }
                    } 
                  }
                }
              } 
            }
          }
        }
      }
    }
  } 
  //set unfinded distances
  for (int i = 0; i < stotal; ++i) 
    closestDistance[i][i] = closestTime[i][i] = 0.0;
  /*
  for (int i = 1; i < sc0; ++i)
    for (int j = 1; j < sc0; ++j) {
      int iId = c0[i]->id,
          jId = c0[j]->id;
      if (closestDistance[iId][jId] < 0.0) {
        closestDistance[iId][jId] = closestDistance[iId][depotId] + closestDistance[depotId][jId];
        closestTime[iId][jId] = closestTime[iId][depotId] + closestTime[depotId][jId];
      }
    }
    */
  return {closestDistance, closestTime};
}

vector<pair<double, double>> utils::calculateClosestsGVRPCustomers (const vector<vector<double>>& gvrpReducedGraph, const vector<const Vertex *>& vertices) {
  const int svertices = vertices.size();
  const Vertex * customerI, 
        * customerJ;
  double cost;
  vector<pair<double, double>> closest (svertices, make_pair(DBL_MAX, DBL_MAX));
  for (int i = 0; i < svertices; ++i) {
    customerI = vertices[i];
    for (int j = 0; j < svertices; ++j) 
      if (i != j) {
        customerJ = vertices[j];
        cost = gvrpReducedGraph[customerI->id][customerJ->id];
        if (closest[i].first > cost) {
          closest[i].second = closest[i].first;
          closest[i].first = cost;
        } else if (closest[i].second > cost) 
          closest[i].second = cost;
      }
    if (closest[i].second == DBL_MAX)
      closest[i].second = closest[i].first;
  } 
  return closest;
}

int utils::calculateGVRP_BPP_NRoutesLB(const Gvrp_instance& gvrp_instance, const vector<const Vertex *>& vertices, const vector<pair<double, double>>& closestsTimes, unsigned int execution_time_limit) {
  const int svertices = vertices.size();
  vector<double> items (svertices); 
  for (int i = 0; i < svertices; ++i) 
    items[i] = vertices[i]->serviceTime + (closestsTimes[i].first + closestsTimes[i].second)/2.0;
  BPP_instance bpp_instance (items, gvrp_instance.timeLimit);
  BPP_model bpp_model (bpp_instance, execution_time_limit);
  return bpp_model.run().first.size();
}

double utils::calculateGvrpInstanceLambda (const Gvrp_instance& gvrp_instance) {
  if (gvrp_instance.afss.size() > 0){
    //calculate lambda
    double lambda = DBL_MAX;
    //min_{v_f \in F} c_{f0} . C
    for (const Vertex& f : gvrp_instance.afss)
      lambda = min(gvrp_instance.distances[f.id][0], lambda);
    return lambda * gvrp_instance.vehicleFuelConsumptionRate;
  }
  return 0.0;  
}

double utils::calculateGvrpInstancePsi (const Gvrp_instance& gvrp_instance) {
  if (gvrp_instance.afss.size() > 1){
    //calculate psi 
    double psi = DBL_MAX;
    //min_{(f, r) \in F : r \neq f} c_{fr} . C
    for (const Vertex& f : gvrp_instance.afss)
      for (const Vertex& r : gvrp_instance.afss)
        if (f.id != r.id)
          psi = min(gvrp_instance.distances[f.id][r.id], psi);
    return psi * gvrp_instance.vehicleFuelConsumptionRate;
  }
  return 0.0;  
}

double utils::calculateGvrpLBByImprovedMST (const vector<const Vertex *>& vertices, const vector<pair<double, double>> closests, const vector<vector<double>>& gvrpReducedGraph) {
  const int svertices = vertices.size();
  const Vertex * nodeI, * nodeJ, * nodeK;
  double bestLB = 0.0, cost;
  //boruvka
  vector<vector<bool>> adjMatrix (svertices, vector<bool> (svertices, false));
  DSU dsu(svertices);
  int nTrees = svertices;
  double MSTCost = 0.0;
  int setI, setJ, setK;
//  cout<<"building mst"<<endl;
  vector<pair<int, int>> bestEdge (svertices, {-1, -1});
  vector<double> bestEdgeCost (svertices, DBL_MAX);
  while (nTrees > 1) {
//    cout<<"\tnTrees: "<<nTrees<<endl;
    //calculate best cuts
    for (int i = 0; i < svertices; ++i) {
      nodeI = vertices[i];
      setI = dsu.findSet(i);
      for (int j = 0; j < svertices; ++j) {
        nodeJ = vertices[j];
        cost = gvrpReducedGraph[nodeI->id][nodeJ->id];
        if (i != j && setI != dsu.findSet(j) && cost < bestEdgeCost[setI]) {
          bestEdgeCost[setI] = cost;
          bestEdge[setI] = {i, j};
        }
      }
    }

    //insert best edges
//    cout<<"Edges: "<<endl;
    for (int i = 0; i < svertices; ++i) {
      setI = dsu.findSet(i);
      //if edge exist
      if (bestEdge[setI].first != -1) {
        int j = bestEdge[setI].first,
            k = bestEdge[setI].second;
        int setK = dsu.findSet(k);
        if (setI != setK) {
          adjMatrix[j][k] = true;
          dsu.join(j, k);
          --nTrees;
          MSTCost += bestEdgeCost[setI];
        }
  //      bestEdgeCost[setI] = bestEdgeCost[setK] = DBL_MAX;
   //     bestEdge[setI] = bestEdge[setK] = {-1, -1};
      }
    }
    bestEdge = vector<pair<int, int>> (svertices, {-1, -1});
    bestEdgeCost = vector<double> (svertices, DBL_MAX);
//    cout<<endl;
//    cout<<"Sets:"<<endl;
    set<int> sets;
    for (int i = 0; i < svertices; ++i) 
      sets.insert(dsu.findSet(i));
    /*
    for (int components : sets) {
      cout<<"\t"<<components<<": ";
      for (int i = 0; i < svertices; ++i) 
        if (dsu.findSet(i) == components)
          cout<<i<<", ";
      cout<<endl;
    }
    */
  }
//  cout<<MSTCost<<endl;


  for (int i = 0; i < svertices; ++i) 
    for (int j = 0; j < svertices; ++j) 
      if (adjMatrix[i][j] == true)
//        cout<<"("<<vertices[i]->id + 1<<", "<<vertices[j]->id + 1<<")";
//  cout<<endl;
  //greedy boruvka
//  cout<<"greedy borukva"<<endl;
  for (int i = 0; i < svertices; ++i) {
//    cout<<"MST without "<<vertices[i]->id + 1<<endl;
//    cout<<"\tRemoved edges";
//    cout<<"\tIn vertex "<<i<<endl;
    nodeI = vertices[i];
    dsu.clean();
//    cout<<"\tremoving "<<i<<" from the tree"<<endl;
    //remove i from the graph   
    for (int j = 0; j < svertices; ++j) {
      nodeJ = vertices[j];
      if (adjMatrix[i][j] || adjMatrix[j][i]) {
        MSTCost -= gvrpReducedGraph[nodeI->id][nodeJ->id];
        adjMatrix[i][j] = adjMatrix[j][i] = false; 
//        cout<<"("<<nodeI->id + 1<<", "<<nodeJ->id + 1<<"): "<<gvrpReducedGraph[nodeI->id][nodeJ->id]<<", ";
      }
    }
//    cout<<endl;
//    cout<<"\tnew MST cost "<<MSTCost<<endl;
//    cout<<"\tnew edges: ";
//    cout<<"\tpopulating DSU"<<endl;
    //populate dsu
    for (int j = 0; j < svertices; ++j) 
      for (int k = 0; k < svertices; ++k) 
        if (adjMatrix[j][k]) {
            dsu.join(j, k);
//            cout<<"("<<vertices[j]->id + 1<<", "<<vertices[k]->id + 1<<")";
        }
//    cout<<endl;
//    cout<<"\tSets:"<<endl;
    set<int> sets;
    for (int i = 0; i < svertices; ++i) 
      sets.insert(dsu.findSet(i));
    /*
    for (int components : sets) {
      cout<<"\t\t"<<components<<": ";
      for (int i = 0; i < svertices; ++i) 
        if (dsu.findSet(i) == components)
          cout<<i<<", ";
      cout<<endl;
    }
    */
    //get number of components
//    cout<<"\tgetting number of components"<<endl;
    nTrees = sets.size();
//    cout<<"\tborukva again"<<endl;
    //borukva
    while (nTrees > 2) {
//      cout<<"\t# trees"<<nTrees<<endl;
//      cout<<"\tnTrees: "<<nTrees<<endl;
      //calculate best cuts
      for (int j = 0; j < svertices; ++j) {
        if (j != i) {
          nodeJ = vertices[j];
          setJ = dsu.findSet(j);
          for (int k = 0; k < svertices; ++k) {
            if (k != i && k != j) {
              nodeK = vertices[k];
              cost = gvrpReducedGraph[nodeJ->id][nodeK->id];
              //cout<<"\t\t"<<j<<" "<<k<<" "<<nodeJ->id<<" "<<nodeK->id<<" "<<cost<<endl;
              if (setJ != dsu.findSet(k) && cost < bestEdgeCost[setJ]) {
                bestEdgeCost[setJ] = cost;
                bestEdge[setJ] = {j, k};
              }
            }
          }
        }
      }
//      cout<<"\tinserted edges: ";
      //insert best edges
//      cout<<"\tInserted edges: "<<endl<<"\t\t";
      for (int j = 0; j < svertices; ++j) {
        setJ = dsu.findSet(j);
        //if edge exist
        if (bestEdge[setJ].first != -1) {
          int k = bestEdge[setJ].first,
              l = bestEdge[setJ].second;
          int setL = dsu.findSet(l);
          if (setJ != setL) {
            adjMatrix[k][l] = true;
            dsu.join(k, l);
            --nTrees;
            MSTCost += bestEdgeCost[setJ];
//            cout<<"("<<vertices[k]->id + 1<<", "<<vertices[l]->id + 1<<"): "<<gvrpReducedGraph[vertices[k]->id][vertices[l]->id]<<", ";
//            cout<<"("<<k<<", "<<j<<"), ";
          }
          bestEdgeCost[setJ] = bestEdgeCost[setL] = DBL_MAX;
          bestEdge[setJ] = bestEdge[setL] = {-1, -1};
        }
      }
//      cout<<endl<<"\tcurrent MST cost "<<MSTCost<<endl;
//      cout<<endl;
//      cout<<"\tSets:"<<endl;
      sets.clear();
      for (int i = 0; i < svertices; ++i) 
        sets.insert(dsu.findSet(i));
      /*
      for (int components : sets) {
        cout<<"\t\t"<<components<<": ";
        for (int i = 0; i < svertices; ++i) 
          if (dsu.findSet(i) == components)
            cout<<i<<", ";
        cout<<endl;
      }
      */
      bestLB = max(closests[i].first + closests[i].second + MSTCost, bestLB);
    }
  }
  return bestLB;
}

double utils::calculateGvrpLBByImprovedMSTTime (const vector<const Vertex *>& vertices, const vector<pair<double, double>> closestsTimes, const vector<vector<double>>& gvrpReducedGraphTimes) {
  double timeLB = utils::calculateGvrpLBByImprovedMST(vertices, closestsTimes, gvrpReducedGraphTimes);
  for (const Vertex * vertice : vertices)
    timeLB += vertice->serviceTime;
  return timeLB;
}

double utils::calculateCustomerMinRequiredTime (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree, const Vertex& customer) {
  //min_{v_f, v_r \in F : t_{T[r]} + t_{fi} + s_i + t_{ir} + t_{T[r]} && e_{fi} + e_{ir} <= \beta} min(min(t_{T[f]} + t_{fi}, t_{ir} + t_{T[r]})) = minAfsFuel 
  double minAfsTime = DBL_MAX;
  for (int f = 0; f < gvrp_afs_tree.f0.size(); f++) 
    for (int r = 0; r < gvrp_afs_tree.f0.size(); r++) {
      double partI = gvrp_afs_tree.times[f] + gvrp_instance.time(gvrp_afs_tree.f0[f]->id, customer.id),
             partII = gvrp_instance.time(customer.id, gvrp_afs_tree.f0[r]->id) + gvrp_afs_tree.times[r];
      if (partI + customer.serviceTime + partII <= gvrp_instance.timeLimit && gvrp_instance.fuel(gvrp_afs_tree.f0[f]->id, customer.id) + gvrp_instance.fuel(customer.id, gvrp_afs_tree.f0[r]->id) <= gvrp_instance.vehicleFuelCapacity) 
        minAfsTime = min(minAfsTime, min(partI, partII));    
    }
  return minAfsTime;
}

double utils::calculateCustomerMinRequiredFuel (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree, const Vertex& customer) {
  //min_{v_f, v_r \in F : t_{T[r]} + t_{fi} + s_i + t_{ir} + t_{T[r]} && e_{fi} + e_{ir} <= \beta} min(e_{fi}, e_{ri}) = minAfsFuel 
  double minAfsFuel = DBL_MAX;
  for (int f = 0; f < gvrp_afs_tree.f0.size(); f++) 
    for (int r = 0; r < gvrp_afs_tree.f0.size(); r++) {
      double partI = gvrp_instance.fuel(customer.id, gvrp_afs_tree.f0[r]->id),
             partII = gvrp_instance.fuel(customer.id, gvrp_afs_tree.f0[f]->id);
      if (gvrp_afs_tree.times[f] + gvrp_instance.time(gvrp_afs_tree.f0[f]->id, customer.id) + customer.serviceTime + gvrp_instance.time(customer.id, gvrp_afs_tree.f0[r]->id) + gvrp_afs_tree.times[r] <= gvrp_instance.timeLimit && partI + partII <= gvrp_instance.vehicleFuelCapacity) 
        minAfsFuel = min(minAfsFuel, min(partI, partII));    
    }
  return minAfsFuel;
}

double utils::calculateCustomerMaxAllowedFuel (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree, const Vertex& customer) {
  return gvrp_instance.vehicleFuelCapacity - utils::calculateCustomerMinRequiredFuel(gvrp_instance, gvrp_afs_tree, customer);
}

list<list<Vertex> > utils::getGvrpConnectedComponents (const Gvrp_instance& gvrp_instance) {
  //setup
  list<list<Vertex>> components; 
  int nNodes = gvrp_instance.distances.size();
  vector<bool> visited (nNodes, false);
  vector<Vertex> all (nNodes);
  set<int> customers;
  list<Vertex> component;
  queue<int> q;
  double phi;
  int curr,
      j;
  for (const Vertex& customer : gvrp_instance.customers) {
    customers.insert(customer.id);
    all[customer.id] = customer;
  }
  for (const Vertex& afs : gvrp_instance.afss)
    all[afs.id] = afs;
  //bfs
  for (const Vertex& customer : gvrp_instance.customers) {
    if (!visited[customer.id]) {
      q.push(customer.id); 
      visited[customer.id] = true;
      while (!q.empty()) {
        curr = q.front();
        q.pop();
        //get phi
        if (customers.count(curr)) {
          phi = DBL_MAX; 
          for (j = 0; j < int(gvrp_instance.distances.size()); j++) 
            if (curr != j)
              phi = min (phi, gvrp_instance.distances[j][curr]);
          phi = gvrp_instance.vehicleFuelCapacity - phi * gvrp_instance.vehicleFuelConsumptionRate;
        } else 
          phi = gvrp_instance.vehicleFuelCapacity;
        //seve in the list
        component.push_back(all[curr]);
        //get neighboring
        for (j = 0; j < int(gvrp_instance.distances.size()); j++) 
          if (!visited[j] && j != gvrp_instance.depot.id && phi >= gvrp_instance.distances[curr][j] * gvrp_instance.vehicleFuelConsumptionRate) {
            visited[j] = true;
            q.push(j);
          }
      }
      components.push_back(component);
      component = list<Vertex> ();
    }
  }
  return components;
}

list<pair<int, int>> utils::get_invalid_edges_1 (const Gvrp_instance& gvrp_instance) {
  if (gvrp_instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 1' only applies for metric instances");
  list<pair<int, int>> edges;
  for (int i = 0; i < gvrp_instance.distances.size(); i++)
    for (int j = 0; j < gvrp_instance.distances.size(); j++) {
      if (gvrp_instance.distances[i][j] * gvrp_instance.vehicleFuelConsumptionRate > gvrp_instance.vehicleFuelCapacity || gvrp_instance.distances[i][j] / gvrp_instance.vehicleAverageSpeed > gvrp_instance.timeLimit) {
        edges.push_back(make_pair(i, j));
      }
    }
  return edges;
}

list<pair<int, int>> utils::get_invalid_edges_2 (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree) {
  if (gvrp_instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 2' only applies for metric instances");
  list<pair<int, int>> edges;
  /*
  Gvrp_feasible_solution_heuristic gvrp_feasible_solution_heuristic (gvrp_instance);
  Gvrp_solution gvrp_solution =  gvrp_feasible_solution_heuristic.run();
  set<int> customers;
  //populate customers set
  for (const Vertex& customer : gvrp_instance.customers)
    customers.insert(customer.id);
  //for each route
  for (const list<Vertex>& route : gvrp_solution.routes) {
    auto second = ++route.begin();
    auto penultimate = --route.rbegin();
    //valid preprocessing
    if (!customers.count(second->id) && !customers.count(penultimate->id)) {
      //get customer
      for (second++; !customers.count(second->id); second++);
      edges.push_back(make_pair(gvrp_instance.depot.id, second->id));
      edges.push_back(make_pair(second->id, gvrp_instance.depot.id));
    }
  }
  return edges;
  */
  for (const Vertex& customer : gvrp_instance.customers) {
    if (gvrp_instance.fuel(customer.id, gvrp_instance.depot.id) > gvrp_instance.vehicleFuelCapacity/2.0) {
      bool valid = false;
      for (int f = 0; f < gvrp_afs_tree.f0.size(); ++f) {
        if (gvrp_instance.fuel(gvrp_afs_tree.f0[f]->id, customer.id) + gvrp_instance.fuel(customer.id, gvrp_instance.depot.id) <= gvrp_instance.vehicleFuelCapacity
            && gvrp_afs_tree.times[f] + gvrp_instance.time(gvrp_afs_tree.f0[f]->id, customer.id) + customer.serviceTime + gvrp_instance.time(customer.id, gvrp_instance.depot.id) <= gvrp_instance.vehicleFuelCapacity) {
          valid = true;
          break;
        }
      }
      if (!valid) {
        edges.push_back({customer.id, gvrp_instance.depot.id});
        edges.push_back({gvrp_instance.depot.id, customer.id});
      }
    }
  }
  return edges;

}

list<pair<int, int>> utils::get_invalid_edges_3 (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree) {
  if (gvrp_instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 3' only applies for metric instances");
  list<pair<int, int>> edges;
  //create induced graph
  const vector<const Vertex *>& f0 = gvrp_afs_tree.f0;
  const vector<int>& pred = gvrp_afs_tree.pred;
  const vector<double>& times = gvrp_afs_tree.times;
  int sf0 = f0.size(),
          r,
          f;
  //get invalid edges
  bool invalidEdge;
  double distance;
  for (const Vertex& i : gvrp_instance.customers) 
    for (const Vertex& j : gvrp_instance.customers) {
      invalidEdge = true;
      //check if edge (i, j) can be feasible
      for (f = 0; f < sf0; f++) 
        if (f0[f]->id == gvrp_instance.depot.id || pred[f] != f) 
          for (r = 0; r < sf0; r++) 
            //if the afss f and r are connected 
            if (f0[r]->id == gvrp_instance.depot.id || pred[r] != r) {
              distance = gvrp_instance.distances[f0[f]->id][i.id] + gvrp_instance.distances[i.id][j.id] + gvrp_instance.distances[j.id][f0[r]->id];
              if (distance * gvrp_instance.vehicleFuelConsumptionRate <= gvrp_instance.vehicleFuelCapacity && (times[f] + (distance/gvrp_instance.vehicleAverageSpeed) + i.serviceTime + j.serviceTime + times[r] <= gvrp_instance.timeLimit)) { 
                invalidEdge = false;
                f = sf0;
                break;
              }
            }
      if (invalidEdge) {
        edges.push_back(make_pair(i.id, j.id));
        edges.push_back(make_pair(j.id, i.id));
      }
    }
  return edges;
}

list<pair<int, int>> utils::get_invalid_edges_4 (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree) {
  if (gvrp_instance.distances_enum != METRIC)
    throw string("The preprocessing 'Invalid edge preprocessing 4' only applies for metric instances");
  list<pair<int, int>> edges;
  //create induced graph
  const vector<const Vertex *>& f0 = gvrp_afs_tree.f0;
  const vector<int>& pred = gvrp_afs_tree.pred;
  const vector<double>& times = gvrp_afs_tree.times;
  int sf0 = f0.size(),
          r,
          f;
  //get invalid edges
  bool invalidEdge;
  double distance;
  for (const Vertex& i : gvrp_instance.customers)
    for (f = 0; f < sf0; f++) {
      invalidEdge = true;
      if (f0[f]->id == gvrp_instance.depot.id || pred[f] != f) 
        for (r = 0; r < sf0; r++) 
          //if the afss f and r are connected 
          if (f0[r]->id == gvrp_instance.depot.id || pred[r] != r) {
            distance = gvrp_instance.distances[f0[f]->id][i.id] + gvrp_instance.distances[i.id][f0[r]->id];
            if (distance * gvrp_instance.vehicleFuelConsumptionRate <= gvrp_instance.vehicleFuelCapacity && times[f] + (distance/gvrp_instance.vehicleAverageSpeed) + i.serviceTime + times[r] <= gvrp_instance.timeLimit) { 
              invalidEdge = false;
//              f = sf0;
              break;
            }
          }
      if (invalidEdge) {
        edges.push_back(make_pair(i.id, f0[f]->id));
        edges.push_back(make_pair(f0[f]->id, i.id));
      }
    }
  return edges;
}

double utils::calculateGvrpLB1 (const vector<pair<double, double>>& closestsDistances) {
  double lb = 0.0;
  for (int i = 0; i < closestsDistances.size(); ++i)
    lb += (closestsDistances[i].first + closestsDistances[i].second)/2.0;
  return lb;
}

double utils::calculateGvrpBestLB (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree) {
  vector<const Vertex *> c0 (gvrp_instance.customers.size() + 1);
  size_t i = 0;
  c0[i] = &gvrp_instance.depot;
  for (const Vertex& customer : gvrp_instance.customers)
    c0[++i] = &customer;
  //reductions
  vector<vector<double>> gvrpReducedGraphDistances = calculateGVRPReducedGraphs (gvrp_instance, gvrp_afs_tree).first;
  //set sol lb
  const auto& closestsDistances = calculateClosestsGVRPCustomers(gvrpReducedGraphDistances, c0);
  return max(calculateGvrpLBByImprovedMST(c0, closestsDistances, gvrpReducedGraphDistances), calculateGvrpLB1(closestsDistances));
}

double utils::calculateGvrpBestNRoutesLB (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree) {
  vector<const Vertex *> c0 (gvrp_instance.customers.size() + 1);
  size_t i = 0;
  c0[i] = &gvrp_instance.depot;
  for (const Vertex& customer : gvrp_instance.customers)
    c0[++i] = &customer;
  //reductions
  vector<vector<double>> gvrpReducedGraphTimes = calculateGVRPReducedGraphs (gvrp_instance, gvrp_afs_tree).second;
  //set sol lb
  const auto& closestsTimes = calculateClosestsGVRPCustomers(gvrpReducedGraphTimes, c0);
  return max(int(ceil(calculateGvrpLBByImprovedMSTTime(c0, closestsTimes, gvrpReducedGraphTimes)/gvrp_instance.timeLimit)), calculateGVRP_BPP_NRoutesLB(gvrp_instance, c0, closestsTimes, 1000000));
}

Gvrp_instance utils::andelmin_bartolini_instance_reader(const string& file_path){
  string line, token;
  ifstream inFile;
  list<Vertex> afss, 
    customers;
  Vertex depot;
  stringstream ss;
  double vehicleFuelCapacity,
         vehicleFuelConsumptionRate,
         timeLimit,
         vehicleAverageSpeed,
         customerServiceTime,
         afsServiceTime;
  int id = 0,
      nCustomers,
      nAFSs;
  //read file
  inFile.open(file_path);
  if (!inFile)
    throw string("Unable to open file ") + string(file_path);
  getline(inFile, line);
  ss.str(line);
  //ignore instance name
  while (getline(ss, token, ' ') && token.size() == 0);
  //ignore # customers
  while (getline(ss, token, ' ') && token.size() == 0);
  nCustomers = stoi(token, NULL);
  //ignore # AFSs 
  while (getline(ss, token, ' ') && token.size() == 0);
  nAFSs = stoi(token, NULL);
  //get route time limit
  while (getline(ss, token, ' ') && token.size() == 0);
  timeLimit = stod(token, NULL);
  //get vehicle fuel capacity
  while (getline(ss, token, ' ') && token.size() == 0);
  vehicleFuelCapacity = stod(token, NULL);
  //get vehicle average speed
  while (getline(ss, token, ' ') && token.size() == 0);
  vehicleAverageSpeed = stod(token, NULL);
  //get customer service time
  while (getline(ss, token, ' ') && token.size() == 0);
  customerServiceTime = stod(token, NULL);
  //get AFS service time
  while (getline(ss, token, ' ') && token.size() == 0);
  afsServiceTime = stod(token, NULL);
  //get vehicle fuel consumption rate 
  //AB2
  if (vehicleAverageSpeed > 0.7)
    vehicleFuelConsumptionRate = 0.2137;
  //AB1
  else
    vehicleFuelConsumptionRate = 0.2;
  //get depot
  ss.clear();
  getline(inFile, line);
  ss.str(line);
  // ignore id
  while (getline(ss, token, ' ') && token.size() == 0);
  // get type
  while (getline(ss, token, ' ') && token.size() == 0);
  // get x
  while (getline(ss, token, ' ') && token.size() == 0);
  depot.x = stod(token, NULL);
  // get y
  while (getline(ss, token, ' ') && token.size() == 0);
  depot.y = stod(token, NULL);
  // get service time
  depot.serviceTime = 0;
  //get AFSs
  for (int i = 0; i < nAFSs; ++i) {
    // read line
    ss.clear();
    getline(inFile, line);
    ss.str(line);
    // ignore id
    while (getline(ss, token, ' ') && token.size() == 0);
    Vertex afs (0, 0, 0, afsServiceTime);
    // get type
    while (getline(ss, token, ' ') && token.size() == 0);
    // get x
    while (getline(ss, token, ' ') && token.size() == 0);
    afs.x = stod(token, NULL);
    // get y
    while (getline(ss, token, ' ') && token.size() == 0);
    afs.y = stod(token, NULL);
    afss.push_back(afs);
  }
  //get customers
  for (int i = 0; i < nCustomers; ++i) {
    // read line
    ss.clear();
    getline(inFile, line);
    ss.str(line);
    // ignore id
    while (getline(ss, token, ' ') && token.size() == 0);
    Vertex customer (stoi(token, NULL), 0, 0, customerServiceTime);
    // ignore type
    while (getline(ss, token, ' ') && token.size() == 0);
    // get x
    while (getline(ss, token, ' ') && token.size() == 0);
    customer.x = stod(token, NULL);
    // get y
    while (getline(ss, token, ' ') && token.size() == 0);
    customer.y = stod(token, NULL);
    customers.push_back(customer);
  }
  // get infeasible customers
  getline(inFile, line);
  getline(inFile, line);
  list<int> infeasibleCustomers;
  if (inFile.peek() != EOF) {
    ss.clear();
    getline(inFile, line);
    ss.str(line);
    while (getline(ss, token, ' '))
      if (token.size() > 0) 
        infeasibleCustomers.push_back(stoi(token, NULL));
  }
  // remove infeasible customers
  for (int oriId : infeasibleCustomers) {
    for (list<Vertex>::iterator i = customers.begin(); i != customers.end(); ++i)
      if (i->id == oriId) {
        customers.erase(i); 
        break;
      }
  }
  inFile.close();
  //calculate distances
  size_t total_size = afss.size() + customers.size() + 1;
  vector<Vertex> vertexes (total_size);
  int j = 0; 
  vertexes[j++] = depot;
  for (list<Vertex>::iterator i = afss.begin(); i != afss.end(); i++) {
    i->id = ++id;
    vertexes[j++] = *i;
  }
  for (list<Vertex>::iterator i = customers.begin(); i != customers.end(); i++) {
    i->id = ++id;
    vertexes[j++] = *i;
  }
  vector<vector<double> > distances(total_size);
  double radiusOfEarth = 4182.44949; // miles, 6371km; 
//  double radiusOfEarth = 6371.0; // miles, 6371km; 
  for (int i = 0; i < total_size; i++){
    distances[i] = vector<double> (total_size);
    for (int j = 0; j < total_size; j++){
//      double PI = 4.0*atan(1.0);
      double dLat1 = vertexes[i].y * (M_PI/180.0);
      double dLat2 = vertexes[j].y * (M_PI/180.0);
      double dLat = dLat1 - dLat2; 
      double dLon = (vertexes[i].x - vertexes[j].x) * (M_PI/180.0); 
      double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(dLat1) * cos(dLat2); 
//      double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
      double c = 2 * asin(sqrt(a));
      distances[vertexes[i].id][vertexes[j].id] = radiusOfEarth * c;
      //      distances[vertexes[i].id][vertexes[j].id] = sqrt(pow(vertexes[i].x - vertexes[j].x, 2) + pow(vertexes[i].y - vertexes[j].y, 2)); 
    }
  }
  return Gvrp_instance(afss, customers, depot, vehicleFuelCapacity, distances, METRIC, customers.size(), timeLimit, vehicleFuelConsumptionRate, vehicleAverageSpeed);
}

//reading
Gvrp_instance utils::matheus_instance_reader(const string& file_path){
  string line, token;
  ifstream inFile;
  list<Vertex> afss, 
    customers;
  Vertex depot;
  stringstream ss;
  double vehicleFuelCapacity,
         vehicleFuelConsumptionRate,
         timeLimit,
         vehicleAverageSpeed;
  int id,
      x,
      y;
  double serviceTime;
  //read file
  inFile.open(file_path);
  if (!inFile)
    throw string("Unable to open file ") + string(file_path);
  //ignore header
  getline(inFile, line);
  //get vehicle average speed 
    //ignore header
  getline(inFile, line);
  ss.str(line);
  getline(ss, token, ';');
  getline(ss, token, ';');
  ss.clear();
  vehicleAverageSpeed = stod(token, NULL);
  //get time limit 
    //ignore header
  getline(inFile, line);
  ss.str(line);
  getline(ss, token, ';');
  getline(ss, token, ';');
  ss.clear();
  timeLimit = stod(token, NULL);
  //get vehicle fuel consumption rate 
    //ignore header
  getline(inFile, line);
  ss.str(line);
  getline(ss, token, ';');
  getline(ss, token, ';');
  ss.clear();
  vehicleFuelConsumptionRate = stod(token, NULL);
  //get beta
    //ignore header
  getline(inFile, line);
  ss.str(line);
  getline(ss, token, ';');
  getline(ss, token, ';');
  ss.clear();
  vehicleFuelCapacity = stod(token, NULL);
  //get depot
    //ignore headers
  getline(inFile, line);
  getline(inFile, line);
  getline(inFile, line);
    //get id
  ss.str(line);
  getline(ss, token, ';');
  depot.id = stoi(token, NULL);
    //get x
  getline(ss, token, ';');
  depot.x = stoi(token, NULL);
    //get y
  getline(ss, token, ';');
  depot.y = stoi(token, NULL);
    //get service time
  getline(ss, token, ';');
  depot.serviceTime = stod(token, NULL);
  ss.clear();
  //get customers
    //ignore headers
  getline(inFile, line);
  getline(inFile, line);
  getline(inFile, line);
  while (line != "AFSs:"){         
    ss.str(line);
    getline(ss, token, ';');
    id = stoi(token, NULL);
    getline(ss, token, ';');
    x = stoi(token, NULL);
    getline(ss, token, ';');
    y = stoi(token, NULL);
    getline(ss, token, ';');
    serviceTime = stod(token, NULL);
    customers.push_back(Vertex(id, x, y, serviceTime));
    ss.clear();
    getline(inFile, line);
  }
  //get afss 
    //ignore header
  getline(inFile, line);
  getline(inFile, line);
  while (true){         
    ss.str(line);
    getline(ss, token, ';');
    id = stoi(token, NULL);
    getline(ss, token, ';');
    x = stoi(token, NULL);
    getline(ss, token, ';');
    y = stoi(token, NULL);
    getline(ss, token, ';');
    serviceTime = stod(token, NULL);
    afss.push_back(Vertex(id, x, y, serviceTime));
    ss.clear();
    if (inFile.peek() == EOF)
      break;
    getline(inFile, line);
  }
  inFile.close();
  //calculate distances
  const int sall = customers.size() + afss.size() + 1;
  vector<vector<double>> distances (sall, vector<double> (sall));
  for (const Vertex& customer : customers) {
    distances[depot.id][customer.id] = floor(sqrt(pow(customer.x - depot.x, 2) + pow(customer.y - depot.y, 2)) + 0.5);
    distances[customer.id][depot.id] = distances[depot.id][customer.id];
    for (const Vertex& customer_ : customers) 
      distances[customer.id][customer_.id] = floor(sqrt(pow(customer.x - customer_.x, 2) + pow(customer.y - customer_.y, 2)) + 0.5);
    for (const Vertex& afs : afss) {
      distances[customer.id][afs.id] = floor(sqrt(pow(customer.x - afs.x, 2) + pow(customer.y - afs.y, 2)) + 0.5);
      distances[afs.id][customer.id] = distances[customer.id][afs.id];
    }
  }
  for (const Vertex& afs : afss) {
    distances[depot.id][afs.id] = floor(sqrt(pow(afs.x - depot.x, 2) + pow(afs.y - depot.y, 2)) + 0.5);
    distances[afs.id][depot.id] = distances[depot.id][afs.id];
    for (const Vertex& afs_ : afss) 
      distances[afs_.id][afs.id] = floor(sqrt(pow(afs.x - afs_.x, 2) + pow(afs.y - afs_.y, 2)) + 0.5);
  }
  return Gvrp_instance(afss, customers, depot, vehicleFuelCapacity, distances, METRIC, customers.size(), timeLimit, vehicleFuelConsumptionRate, vehicleAverageSpeed);
}

Gvrp_instance utils::erdogan_instance_reader(const string file_path){
  double time_customer = 30,
          time_afss = 15;
  int id = 0,
      customers_size,
      afss_size,
      total_size;
  string buff, 
         type, 
         x, 
         y,
         line;
  ifstream inFile;
  list<Vertex> afss, 
    customers;
  stringstream ss;
  Vertex depot;
  int maxRoutes;
  double vehicleFuelCapacity,
         vehicleFuelConsumptionRate,
         timeLimit,
         vehicleAverageSpeed;
  //read file
  inFile.open(file_path);
  if (!inFile)
    throw string("Unable to open file ") + string(file_path);
  //ignore header
  getline(inFile, line);
  //get depot
  getline(inFile, line);
  ss.str(line);
  //ignore id
  ss>>buff;
  ss>>type;
  ss>>x;
  ss>>y;
  depot = Vertex(id++, stod(x, NULL), stod(y, NULL));
  //get afs's
  getline(inFile, line);
  ss.clear();
  ss.str(line);
  //ignore id
  ss>>buff;
  //get type
  ss>>type;
  ss>>x;
  ss>>y;
  while (type == "f"){         
    afss.push_back(Vertex(id++, stod(x, NULL), stod(y, NULL), time_afss));
    getline(inFile, line);
    ss.clear();
    ss.str(line);
    //ignore id
    ss>>buff;
    //get type
    ss>>type;
    ss>>x;
    ss>>y;
  }
  //get customers
  while (type == "c"){         
    customers.push_back(Vertex(id++, stod(x, NULL), stod(y, NULL), time_customer));
    getline(inFile, line);
    if (line.empty() || line == "\n"|| line == "\r")
      break;
    ss.clear();
    ss.str(line);
    //ignore id
    ss>>buff;
    //get type
    ss>>type;
    ss>>x;
    ss>>y;
  }
  //get beta
  getline(inFile, line);
  ss.clear();
  ss.str(line);  
  while (!ss.eof())
    ss>>buff;  
  vehicleFuelCapacity = stod(buff.substr(1), NULL);
  //get vehicle fuel consumption rate 
  getline(inFile, line);
  ss.clear();
  ss.str(line);  
  while (!ss.eof())
    ss>>buff;  
  vehicleFuelConsumptionRate = stod(buff.substr(1), NULL);
  //get time limit 
  getline(inFile, line);
  ss.clear();
  ss.str(line);  
  while (!ss.eof())
    ss>>buff;  
  timeLimit = stod(buff.substr(1), NULL) * 60;
  //get vehicle average speed 
  getline(inFile, line);
  ss.clear();
  ss.str(line);  
  while (!ss.eof())
    ss>>buff;  
  vehicleAverageSpeed = stod(buff.substr(1), NULL);
  //get number of routes 
  /*
  getline(inFile, line);
  ss.clear();
  ss.str(line);  
  while (!ss.eof())
    ss>>buff;  
  maxRoutes = stoi(buff.substr(1), NULL);
  */
  inFile.close();
  //save data
  customers_size = customers.size();
  maxRoutes = customers_size;
  afss_size = afss.size();
  total_size = customers_size + afss_size + 1;
  //calculate distances
  vector<Vertex> vertexes (total_size);
  int j = 0; 
  vertexes[j++] = depot;
  for (list<Vertex>::iterator i = afss.begin(); i != afss.end(); i++)
    vertexes[j++] = *i;
  for (list<Vertex>::iterator i = customers.begin(); i != customers.end(); i++)
    vertexes[j++] = *i;
  vector<vector<double> > distances(total_size);
  double radiusOfEarth = 4182.44949; // miles, 6371km; 
//  double radiusOfEarth = 6371.0; // miles, 6371km; 
  for (int i = 0; i < total_size; i++){
    distances[i] = vector<double> (total_size);
    for (int j = 0; j < total_size; j++){
//      double PI = 4.0*atan(1.0);
      double dLat1 = vertexes[i].y * (M_PI/180.0);
      double dLat2 = vertexes[j].y * (M_PI/180.0);
      double dLat = dLat1 - dLat2; 
      double dLon = (vertexes[i].x - vertexes[j].x) * (M_PI/180.0); 
      double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(dLat1) * cos(dLat2); 
//      double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
      double c = 2 * asin(sqrt(a));
      distances[vertexes[i].id][vertexes[j].id] = radiusOfEarth * c;
      //      distances[vertexes[i].id][vertexes[j].id] = sqrt(pow(vertexes[i].x - vertexes[j].x, 2) + pow(vertexes[i].y - vertexes[j].y, 2)); 
    }
  }
  timeLimit = 645;
  vehicleAverageSpeed = vehicleAverageSpeed/60.0;
  return Gvrp_instance(afss, customers, depot, vehicleFuelCapacity, distances, METRIC, maxRoutes, timeLimit, vehicleFuelConsumptionRate, vehicleAverageSpeed);
}

Gvrp_solution utils::read_gvrp_solution (const string& file_path, const Gvrp_instance& gvrp_instance) {
  //setup
  map<int, Vertex> ids;
  string buff, 
         line;
  ifstream inFile;
  list<list<Vertex>> routes;
  list<Vertex> route;
  stringstream ss;
  //build map of ids
  for (const Vertex& customer : gvrp_instance.customers)
    ids[customer.id] = customer;
  for (const Vertex& afs : gvrp_instance.afss)
    ids[afs.id] = afs;
  ids[gvrp_instance.depot.id] = gvrp_instance.depot;
  //read file
  inFile.open(file_path);
  if (!inFile)
    throw string("Unable to open file ") + string(file_path);
  while (true) {
    getline(inFile, line);
    ss.str(line);
    ss>>buff;
    if (buff != "Route")
      break;
    ss>>buff;
    //get nodes
    for (ss>>buff; '0' <= buff[0] && buff[0] <= '9'; ss>>buff) 
      route.push_back(ids[stoi(buff, NULL)]);
    routes.push_back(route);
    route.clear();
  }
  return Gvrp_solution (routes, gvrp_instance);
}

list<string> utils::listFilesFromDir(string path) {
  list<string> files;
  struct dirent *entry;
  DIR *dir = opendir(path.c_str());
  if (dir == NULL) 
    throw string("Error in accessing ") + path;
  while ((entry = readdir(dir)) != NULL) 
    if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
      files.push_back (entry->d_name);
  closedir(dir);
  files.sort();
  return files; 
}

/*
vector<vector<int>> utils::read_uchoa_vrp_solution (const string& file_path) {
  ifstream inFile;
  //read file
  inFile.open(file_path);
  if (!inFile)
    throw string("Unable to open file ") + string(file_path);
  //setup
  int nRoutes, 
      nVertexes;
  string buff, 
         line;
  stringstream ss;
  //ignore first line
  getline(inFile, line);
  //get # of routes
  getline(inFile, line);
  ss.str(line);
  ss>>buff;
  ss.clear();
  nRoutes = stoi(buff, NULL);
  vector<vector<int>> routes (nRoutes);
  //ignore two lines
  getline(inFile, line);
  getline(inFile, line);
  //get routes
  for (int j = 0; j < nRoutes; ++j) {
    getline(inFile, line);
    ss.str(line);
    ss>>buff;
    ss>>buff;
    ss>>buff;
    ss>>buff;
    ss>>buff;
    ss>>buff;
    nVertexes = stoi(buff, NULL);
    vector<int> route (nVertexes);
    for (int i = 0; i < nVertexes; ++i) {
      ss>>buff;
      route[i] = stoi(buff, NULL);
    }
    routes[j] = route;
    ss.clear();
  }
  inFile.close();
  return routes;
}
*/

Vrp_instance utils::read_uchoa_vrp_instance (const string& file_path) {
  int id = 0,
      nVertexes;
  string buff, 
         x, 
         y,
         line;
  ifstream inFile;
  list<Vertex> vertexes;
  stringstream ss;
  //read file
  inFile.open(file_path);
  if (!inFile)
    throw string("Unable to open file ") + string(file_path);
  //ignore header
  for (int i = 0; i < 4; i++)
    getline(inFile, line);
  //get # vertexes
  ss.str(line);
  ss>>buff;
  ss>>buff;
  ss>>buff;
  nVertexes = stoi(buff, NULL);
  ss.clear();
  //ignore header
  for (int i = 0; i < 3; i++)
    getline(inFile, line);
  //get vertexes 
  for (int i = 0; i < nVertexes; i++) {
    getline(inFile, line);
    ss.str(line);
    //ignore id
    ss>>buff;
    //get axis
    ss>>x;
    ss>>y;
    vertexes.push_back(Vertex(id++, stod(x, NULL), stod(y, NULL)));
    ss.clear();
  } 
  //calculate distances
  vector<vector<double> > distances(nVertexes);
  int i = 0;
  for (Vertex a : vertexes) {
    distances[i] = vector<double> (nVertexes);
    for (Vertex b : vertexes) 
      distances[a.id][b.id] = floor(sqrt (pow(a.x - b.x, 2) + pow(a.y - b.y, 2)) + 0.5);
    i++;
  }
  //get depot
  Vertex depot = Vertex(*vertexes.begin());
  //    Vertex depot = Vertex (vertexes.begin()->id, vertexes.begin()->x, vertexes.begin()->y);
  //remove header
  vertexes.erase (vertexes.begin());
  return Vrp_instance (vertexes, depot, distances, METRIC, vertexes.size());
} 

Vrp_instance utils::read_augerat_vrp_1995_setP_instance (const string& file_path) {
  int id = 0,
      nVertexes;
  string buff, 
         x, 
         y,
         line;
  ifstream inFile;
  list<Vertex> vertexes;
  stringstream ss;
  //read file
  inFile.open(file_path);
  if (!inFile)
    throw string("Unable to open file ") + string(file_path);
  //ignore header
  for (int i = 0; i < 4; i++)
    getline(inFile, line);
  //get # vertexes
  ss.str(line);
  ss>>buff;
  ss>>buff;
  ss>>buff;
  nVertexes = stoi(buff, NULL);
  ss.clear();
  //ignore header
  for (int i = 0; i < 3; i++)
    getline(inFile, line);
  //get vertexes 
  for (int i = 0; i < nVertexes; i++) {
    getline(inFile, line);
    ss.str(line);
    //ignore id
    ss>>buff;
    //get axis
    ss>>x;
    ss>>y;
    vertexes.push_back(Vertex(id++, stod(x, NULL), stod(y, NULL)));
    ss.clear();
  } 
  //calculate distances
  vector<vector<double> > distances(nVertexes);
  int i = 0;
  for (Vertex a : vertexes) {
    distances[i] = vector<double> (nVertexes);
    for (Vertex b : vertexes) 
      distances[a.id][b.id] = floor(sqrt (pow(a.x - b.x, 2) + pow(a.y - b.y, 2)) + 0.5);
    i++;
  }
  //get depot
  Vertex depot = Vertex(*vertexes.begin());
  //    Vertex depot = Vertex (vertexes.begin()->id, vertexes.begin()->x, vertexes.begin()->y);
  //remove header
  vertexes.erase (vertexes.begin());
  return Vrp_instance (vertexes, depot, distances, METRIC, vertexes.size());
}

void utils::removeDistanceSymmetries (vector<vector<double>>& distances) {
  const size_t n = distances.size();
  for (size_t i = 0; i < n; ++i) 
    for (size_t j = i + 1; j < n; ++j) 
      distances[i][j] -= 1e-2;
}
