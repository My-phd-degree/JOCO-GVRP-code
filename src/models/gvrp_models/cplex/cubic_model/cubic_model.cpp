#include "utils/util.hpp"
#include "models/vertex.hpp"
#include "models/cplex/mip_solution_info.hpp"
#include "models/cplex/depth_node_callback.hpp"
#include "models/distances_enum.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/gvrp_afs_tree.hpp"
#include "models/gvrp_models/cplex/gvrp_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/greedy_lp_heuristic.hpp"
#include "models/gvrp_models/cplex/cubic_model/subcycle_lazy_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/subcycle_user_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/invalid_edge_preprocessing.hpp"
#include "models/gvrp_models/cplex/cubic_model/invalid_edge_preprocessing_2.hpp"
#include "models/gvrp_models/cplex/cubic_model/invalid_edge_preprocessing_3.hpp"
#include "models/gvrp_models/cplex/cubic_model/invalid_edge_preprocessing_4.hpp"
#include "models/gvrp_models/cplex/cubic_model/lazy_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/user_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/extra_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/preprocessing.hpp"

#include <list>
#include <set>
#include <vector>
#include <queue>
#include <stdlib.h>
#include <exception>
#include <sstream>
#include <time.h>
#include <ilcplex/ilocplex.h>

ILOSTLBEGIN

using namespace std;
using namespace utils;
using namespace models;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex;
using namespace models::gvrp_models::cplex::cubic_model;

Cubic_model::Cubic_model(const Gvrp_instance& instance, unsigned int _time_limit): Gvrp_model(instance, time_limit), afss(unordered_set<int> (instance.afss.size())), nGreedyLP(0), BPPTimeLimit(100000000), levelSubcycleCallback(0), nPreprocessings1(0), nPreprocessings2(0), nPreprocessings3(0), nPreprocessings4(0), nImprovedMSTNRoutesLB(0), nBPPNRoutesLB(0), nImprovedMSTNRoutesLBLazy(0), nBPPNRoutesLBLazy(0), RELAXED(false)  {
  if (instance.distances_enum != METRIC)
    throw string("Error: The compact model requires a G-VRP instance with symmetric or metric distances");
  //fill all and customers
  for (const Vertex& customer : instance.customers) {
    all[customer.id] = &customer;
    customers.insert(customer.id);
  }
  for (const Vertex& afs : instance.afss) {
    all[afs.id] = &afs;
    afss.insert(afs.id);
  }
  all[instance.depot.id] = &instance.depot;
  //reductions
  const auto& gvrpReducedGraphs = calculateGVRPReducedGraphs (instance, *gvrp_afs_tree);
  vector<vector<double>> gvrpReducedGraphDistances = gvrpReducedGraphs.first;
  gvrpReducedGraphTimes = gvrpReducedGraphs.second;

  //set sol lb
  vector<const Vertex *> c0 (instance.customers.size() + 1);
  c0[0] = &instance.depot;
  int i = 1;
  for (const Vertex& customer : instance.customers) {
    c0[i] = &customer;
    ++i;
  }
  const auto& closestsDistances = calculateClosestsGVRPCustomers(gvrpReducedGraphDistances, c0);
  solLB = max(calculateGvrpLBByImprovedMST(c0, closestsDistances, gvrpReducedGraphDistances), calculateGvrpLB1(closestsDistances));
  //set n routes lb
  const auto& closestsTimes = calculateClosestsGVRPCustomers(gvrpReducedGraphTimes, c0);
  nRoutesLB = max(int(ceil(calculateGvrpLBByImprovedMSTTime(c0, closestsTimes, gvrpReducedGraphTimes)/instance.timeLimit)), calculateGVRP_BPP_NRoutesLB(instance, c0, closestsTimes, 1000000));
  //customer min required fuel
  customersMinRequiredFuel = unordered_map<int, double> (c0.size() - 1);
  for (const Vertex& customer : instance.customers) 
    customersMinRequiredFuel[customer.id] = calculateCustomerMinRequiredFuel(instance, *gvrp_afs_tree, customer);
  //lazies
  lazy_constraints.push_back(new Subcycle_lazy_constraint(*this));
  //user cuts
  user_constraints.push_back(new Subcycle_user_constraint(*this));
  //preprocessings
  preprocessings.push_back(new Invalid_edge_preprocessing(*this));
  preprocessings.push_back(new Invalid_edge_preprocessing_2(*this));
  preprocessings.push_back(new Invalid_edge_preprocessing_3(*this));
  preprocessings.push_back(new Invalid_edge_preprocessing_4(*this));
  //heuristic callbacks
//  heuristic_callbacks.push_back(new Greedy_lp_heuristic(*this));
}

Cubic_model::~Cubic_model() {
  for (Lazy_constraint * lazy_constraint : lazy_constraints)
    delete lazy_constraint;
  for (User_constraint * user_constraint : user_constraints)
    delete user_constraint;
  for (Preprocessing * preprocessing : preprocessings)
    delete preprocessing;
  for (Extra_constraint * extra_constraint : extra_constraints)
    delete extra_constraint;
}

pair<Gvrp_solution, Mip_solution_info> Cubic_model::run(){
  //setup
  stringstream output_exception;
  Mip_solution_info mipSolInfo;
  try {
//    cout<<"Creating variables"<<endl;
    createVariables();
//    cout<<"Creating objective function"<<endl;
    createObjectiveFunction();
//    cout<<"Creating model"<<endl;
    createModel();
//    cout<<"Setting parameters"<<endl;
    setCustomParameters();
//    cout<<"Solving model"<<endl;
    struct timespec start, finish;
    double elapsed;
    clock_gettime(CLOCK_MONOTONIC, &start);
    if ( !cplex.solve() ) {
//      env.error() << "Failed to optimize LP." << endl;
      mipSolInfo = Mip_solution_info(-1, cplex.getStatus(), -1, -1);
      endVars();
//      env.end();
      throw mipSolInfo;
    }
    clock_gettime(CLOCK_MONOTONIC, &finish);
    elapsed = (finish.tv_sec - start.tv_sec) + (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
    mipSolInfo = Mip_solution_info(cplex.getMIPRelativeGap(), cplex.getStatus(), elapsed, cplex.getObjValue());
    if (RELAXED) {
      endVars ();
      throw mipSolInfo;
    }
//    cplex.exportModel("cplexcpp.lp");
//    env.out() << "Solution value = " << cplex.getObjValue() << endl;
//    cout<<"Getting x values"<<endl;
    fillX_vals();
//    cout<<"Creating GVRP solution"<<endl;
    createGvrp_solution();
    endVals();
    endVars();
//    env.end();
    return make_pair(*solution, mipSolInfo);
  } catch (IloException& e) {
    output_exception<<"Concert exception caught: " << e<<endl;
    throw output_exception.str();
  } catch (string s) {
    throw s;
  }
}

void Cubic_model::createVariables(){
  x = Matrix3DVar (env, instance.maxRoutes);
  e = IloNumVarArray (env, all.size(), 0, instance.vehicleFuelCapacity, IloNumVar::Float);
  try {
    //setting names
    stringstream nameStream;
    for (int customer : customers){
      nameStream<<"e["<<customer<<"]";
      const string name = nameStream.str();
      e[customer].setName(name.c_str());
      nameStream.clear();
      nameStream.str("");
    }
    //x var
    for (int k = 0; k < instance.maxRoutes; k++){
      x[k] = IloArray<IloNumVarArray> (env, all.size());
      for (const pair<int, const Vertex *>& p : all){
        int i = p.first;
        x[k][i] = IloNumVarArray(env, all.size(), 0, 1, RELAXED ? IloNumVar::Float : IloNumVar::Int);
        //setting names
        for (const pair<int, const Vertex *>& p1 : all){
          int j = p1.first;
          nameStream<<"x["<<k<<"]["<<i<<"]["<<j<<"]";
          const string name_x = nameStream.str();
          x[k][i][j].setName(name_x.c_str());
          nameStream.clear();
          nameStream.str("");
        }
      }
    }
  }catch (IloException& e) {
    throw e;
  } catch(...){
    throw string("Error in creating variables");
  }
}

void Cubic_model::createObjectiveFunction() {
//objective function
  try{
    IloExpr fo (env);
    for (int k = 0; k < instance.maxRoutes; k++)
      for (const pair<int, const Vertex *>& p : all){
        int i = p.first;
        for (const pair<int, const Vertex *>& p1 : all){
          int j = p1.first;
          fo +=  instance.distances[i][j] * x[k][i][j];
        }
      }  
    model = IloModel (env);
    model.add(IloMinimize(env, fo));
  } catch (IloException& e) {
    throw e;
  } catch(...){
    throw string("Error in creating the objective function");
  }
}

void Cubic_model::createModel() {
  try {
    //preprocessing conditions
    for (Preprocessing* preprocessing : preprocessings)
      preprocessing->add();
    //setup
    int depot = instance.depot.id;
    double beta = instance.vehicleFuelCapacity;
    double T = instance.timeLimit;
    //constraints
    IloExpr expr(env),
            expr1(env);    
    IloConstraint c;
    stringstream constraintName;
    //\sum_{v_j \in V} x_{ij}^k = \sum_{v_j \in V} x_{ji}^k, \forall v_i \in V, \forall k \in M
    for (int k = 0; k < instance.maxRoutes; k++)
      for (const pair<int, const Vertex *>& p : all){
        int i = p.first;
        for (const pair<int, const Vertex *>& p1 : all){
          int j = p1.first;
          expr += x[k][i][j];
          expr1 += x[k][j][i];
        }
        c = IloConstraint (expr == expr1);
        constraintName<<"#entering edges == #exiting edges in "<<i<<" in route "<<k;
        c.setName(constraintName.str().c_str());
        model.add(c);
        expr1.end();
        expr.end();
        expr = IloExpr(env);
        expr1 = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }    
    //\sum_{v_i \in V} x_{0i}^k \leqslant 1, \forall k in M
    for (int k = 0; k < instance.maxRoutes; k++){
      for (const pair<int, const Vertex *>& p : all){
        int i = p.first;
        expr += x[k][depot][i];
      }
      c = IloConstraint (expr <= 1);
      constraintName<<"route "<<k<<" must be used at most once";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{k \in M} \sum_{v_j \in V} x_{ij}^{k} = 1, \forall v_i \in C
    for (const Vertex& customer :instance.customers){
      int i = customer.id;
      for (int k = 0; k < instance.maxRoutes; k++){
        for (const pair<int, const Vertex *>& p1 : all){
          int j = p1.first;
          if (i != j) 
            expr += x[k][i][j];
        }
      }
      c = IloConstraint (expr == 1);
      constraintName<<"customer "<<i<<" must be visited exactly once";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{k \in M} \sum_{v_j \in V} x_{0j}^k \leqslant m
    for (int k = 0; k < instance.maxRoutes; k++)
      for (const pair<int, const Vertex *>& p : all)
        expr += x[k][depot][p.first];
    c = IloConstraint (expr <= instance.maxRoutes);
    c.setName("# routes upper bound");
    model.add(c);
    expr.end();
    expr = IloExpr(env);
    //e_j \leq e_i - c_{ij} x_{ij}^k + \beta (1 - x_{ij}^k) + e_{ji} x_{ji}^k, \forall v_j \in C,\forall v_i \in V, \forall k \in M
    for (int k = 0; k < instance.maxRoutes; k++)
      for (int i : customers) {
        for (int j : customers) {
          expr = e[i] - x[k][i][j] * instance.fuel(i, j) + beta * (1 - x[k][i][j]) + x[k][j][i] * instance.fuel(j, i);
          c = IloConstraint (e[j] <= expr);
          constraintName<<"e_"<<j<<" update fuel level";
          c.setName(constraintName.str().c_str());
          model.add(c);
          expr.end();
          expr = IloExpr (env);
          constraintName.clear();
          constraintName.str("");
        }
      }
    //UB_E^i \geq e_j \leq LB_E^i, \forall v_j \in C
    for (int j : customers) {
      //ub
      c = IloConstraint (e[j] <= instance.vehicleFuelCapacity - customersMinRequiredFuel[j]);
      constraintName<<"e_"<<j<<" fuel level upper bound";
      c.setName(constraintName.str().c_str());
      model.add(c);
      constraintName.clear();
      constraintName.str("");
      //lb
      c = IloConstraint (e[j] >= customersMinRequiredFuel[j]);
      constraintName<<"e_"<<j<<" fuel level lower bound";
      c.setName(constraintName.str().c_str());
      model.add(c);
      constraintName.clear();
      constraintName.str("");
    }
    //e_i \geq e_{ij} x_{ij}^k, \forall v_i \in C, \forall v_j \in F_0, \forall k \in M
      for (int i : customers)
        for (const pair<int, const Vertex *>& p1 : all) {
          int j = p1.first;
          if (!customers.count(j)) {
            for (int k = 0; k < instance.maxRoutes; k++)
              expr += instance.distances[i][j] * x[k][i][j] * instance.vehicleFuelConsumptionRate;
            c = IloConstraint (e[i] >= expr);
            constraintName<<"e_"<<i<<" mandatory fuel to "<<j;
            c.setName(constraintName.str().c_str());
            model.add(c);
            expr.end();
            expr = IloExpr(env);
            constraintName.clear();
            constraintName.str("");
          }
        }
    //e_i \leq e_{ji} x_{ji}^k, \forall v_i \in C, \forall v_j \in F_0, \forall k \in M
      for (int i : customers)
        for (const pair<int, const Vertex *>& p1 : all){
          int j = p1.first;
          if (!customers.count(j)) {
            for (int k = 0; k < instance.maxRoutes; k++)
              expr -= instance.distances[j][i] * x[k][j][i] * instance.vehicleFuelConsumptionRate;
            expr += instance.vehicleFuelCapacity;
            c = IloConstraint (e[i] <= expr);
            constraintName<<"e_"<<i<<" mandatory fuel to "<<j;
            c.setName(constraintName.str().c_str());
            model.add(c);
            expr.end();
            expr = IloExpr(env);
            constraintName.clear();
            constraintName.str("");
          }
        }
    //x_{ij}^k c_{ij} \leq \beta, \forall v_i, \forall v_j \in V, \forall k \in M
    for (int k = 0; k < instance.maxRoutes; k++)
      for (const pair<int, const Vertex *>& p : all){
        int i = p.first;
        for (const pair<int, const Vertex *>& p1 : all){
          int j = p1.first;
          expr = instance.distances[i][j] * x[k][i][j] * instance.vehicleFuelConsumptionRate;
          c = IloConstraint (expr <= beta);
          constraintName<<"disabling infeasible edge ("<<i<<", "<<j<<") again";
          c.setName(constraintName.str().c_str());
          model.add(c);
          expr.end();
          expr = IloExpr(env);
          constraintName.clear();
          constraintName.str("");
        }
      }
    //\sum_{(i, j) \in E} x_{ij}^k ((c_{ij} / S) + time(v_i) )\leq T, \forall k \in M
    for (int k = 0; k < instance.maxRoutes; k++){
      for (const pair<int, const Vertex *>& p : all){
        int i = p.first;
        for (const pair<int, const Vertex *>& p1 : all){
          int j = p1.first;
          expr += x[k][i][j] * (instance.time(i, j) + p.second->serviceTime);
        }
      }
      c = IloConstraint (expr <= T);
      constraintName<<"route "<<k<<" time limit constraint";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //route fuel lb 1
    for (int k = 0; k < instance.maxRoutes; ++k) 
      for (int f_ : afss)
        for (const pair<int, const Vertex *>& p1 : all){
          int i_ = p1.first;
          for (const pair<int, const Vertex *>& p2 : all){
            int i = p2.first;
            for (int f_ : afss)
              expr -= x[k][i][f_] * psi;
            for (const pair<int, const Vertex *>& p3 : all){
              int j = p3.first;
              expr += x[k][i][j] * instance.fuel(i, j);
            }
          }
          c = IloConstraint (expr >= x[k][f_][i_] * (-psi + 2 * lambda));
          constraintName<<"route "<<k<<" fuel lb 1";
          c.setName(constraintName.str().c_str());
          model.add(c);
          expr.end();
          expr = IloExpr(env);
          constraintName.clear();
          constraintName.str("");
        }
    //route fuel lb 2
    for (int k = 0; k < instance.maxRoutes; ++k) 
      for (int f_ : afss)
        for (const pair<int, const Vertex *>& p1 : all){
          int i_ = p1.first;
          for (const pair<int, const Vertex *>& p2 : all){
            int i = p2.first;
            for (int f_ : afss)
              expr -= x[k][i][f_] * instance.vehicleFuelCapacity / 2.0;
            for (const pair<int, const Vertex *>& p3 : all){
              int j = p3.first;
              expr += x[k][i][j] * instance.fuel(i, j);
            }
          }
          c = IloConstraint (expr >= x[k][f_][i_] * alpha);
          constraintName<<"route "<<k<<" fuel lb 1";
          c.setName(constraintName.str().c_str());
          model.add(c);
          expr.end();
          expr = IloExpr(env);
          constraintName.clear();
          constraintName.str("");
        }
    //n afs visits UB per route
    for(int k = 0; k < instance.maxRoutes; ++k) {
      for (int afs : afss) {
        for (const pair<int, const Vertex *>& p : all) {
          expr += x[k][afs][p.first];
          for (int customer : customers)
            expr -= x[k][customer][p.first];
        }
        c = IloConstraint (expr <= 1);
        constraintName<<"afs "<<afs<<" route "<<k<<" n visit ub";
        c.setName(constraintName.str().c_str());
        model.add(c);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    }
    //route i + 1 is used only if route i is used
    for (int k = 1; k < instance.maxRoutes; ++k) {
      for (const pair<int, const Vertex *>& p : all) 
        for (const pair<int, const Vertex *>& p1 : all) 
          expr += x[k][p.first][p1.first] - x[k - 1][p.first][p1.first];
      c = IloConstraint (expr <= 0);
      constraintName<<"route "<<k<<" is used only if route "<<k - 1<<" is used";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //n routes LB
    for (int k = 0; k < instance.maxRoutes; ++k) 
      for (const pair<int, const Vertex *>& p2 : all){
        int i = p2.first;
        expr += x[k][0][i];
      }
    c = IloConstraint (expr >= nRoutesLB);
    c.setName("nRoutes LB");
    model.add(c);
    expr.end();
    expr = IloExpr(env);
    //solution lb
    for (int k = 0 ; k < instance.maxRoutes; ++k)
      for (const pair<int, const Vertex *>& p : all){
        int i = p.first;
        for (const pair<int, const Vertex *>& p1 : all){
          int j = p1.first;
          expr += x[k][i][j] * instance.distances[i][j];
        }
      }
    c = IloConstraint (expr >= solLB);
    c.setName("solution LB");
    model.add(c);
    expr.end();
    expr = IloExpr(env);
    //extra constraints
    for (Extra_constraint* extra_constraint : extra_constraints) 
      extra_constraint->add();







    /*
    list<list<int>> routesIds = {
      {0, 19, 5, 14, 2, 16, 2, 0},
      {0, 6, 2, 17, 18, 3, 2, 1, 0},
      {0, 9, 7, 2, 12, 15, 11, 4, 1, 10, 2, 13, 8, 0},
    };
    list<list<Vertex>> routes;
    for (const list<int>& routeId : routesIds) {
      list<Vertex> route;
      for (int id : routeId)
        route.push_back(Vertex(*all[id]));
      routes.push_back(route);
    }
    Gvrp_solution gvrp_sol (routes, instance);
    cout<<gvrp_sol<<endl;
    int k = 0;
    for (const list<Vertex>& route : routes) {
      for (auto prev = route.begin(), curr = ++route.begin(); curr != route.end(); prev = curr, ++curr)
        model.add(x[k][prev->id][curr->id] == 1);
      ++k;
    }
    */















    //init
    cplex = IloCplex(model);
    //lazy constraints
    for (Lazy_constraint * lazy_constraint : lazy_constraints)
      cplex.use(lazy_constraint);
    //user cuts
    for (User_constraint * user_constraint : user_constraints)
      cplex.use(user_constraint);
    //heuristic callbacks
    for (Heuristic_callback * heuristic_callback : heuristic_callbacks)
      cplex.use(heuristic_callback);
    //extra steps
    extraStepsAfterModelCreation();
    //depth node callback
    depth_node_callback = new Depth_node_callback(env);
    cplex.use(depth_node_callback);
  } catch (IloException& e) {
    throw e;
  } catch (string s) {
    throw s;
  }
}

void Cubic_model::extraStepsAfterModelCreation() {
  //
}

void Cubic_model::setCustomParameters(){
  try{
    setParameters();
    //although this parameter is not necessary, it is being defined to standarize the experiments
    cplex.setParam(IloCplex::Param::Preprocessing::Linear, 0);
    //although this parameter parameter is defined as default (since this formulation makes use of lazy constraint callback), it is being defined to standarize the experiments
    cplex.setParam(IloCplex::Param::Threads, 1);
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in setting parameters");
  }
}

void Cubic_model::fillX_vals(){
  //getresult
  try{
    x_vals = Matrix3DVal (env, instance.maxRoutes);
    for (int k = 0; k < instance.maxRoutes; k++) { 
      x_vals[k] = IloArray<IloNumArray> (env, all.size());
      for (const pair<int, const Vertex *>& p : all){
        int i = p.first;
        x_vals[k][i] = IloNumArray (env, all.size(), 0, 1, IloNumVar::Int);
        cplex.getValues(x_vals[k][i], x[k][i]);
      }
    }
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting solution");
  }
}

void Cubic_model::createGvrp_solution(){
  /*
  //print x vals
  for (int k = 0; k < instance.maxRoutes; k++){ {
    //columns
    cout<<" ";
    for (auto p : all){
      if (p.first <= 9)
        cout<<" ";
      cout<<" "<<p.first;
    }
    cout<<endl;
    //content
    for (auto p : all){
      cout<<p.first;
      if (p.first <= 9)
        cout<<" ";
      for (auto p1 : all){
        cout<<" "<<int(x_vals[k][p.first][p1.first])<<" ";
      }
      cout<<endl;
    }
  }
  */
  try{
    list<list<Vertex> > routes;
    //dfs
    int depot = instance.depot.id;
    int curr,
        next;    
    for (int k = 0; k < instance.maxRoutes; k++) {
      curr = depot;
      next = depot;
      //checking if the route is used
      //get remaining nodes (if exists)
      set<int> routeAfssSet;
      queue<int> routeAfss;
      list<Vertex> route;
      route.push_back(*all[curr]);
      do {
        //get new neighborhood
        for (auto it  = all.rbegin(); it != all.rend(); ++it){
          int i = it->first;
          if (x_vals[k][curr][i] > INTEGRALITY_TOL){
            if (!customers.count(i) && i != depot && !routeAfssSet.count(i)) {
              routeAfssSet.insert(i);
              routeAfss.push(i);
            }
            next = i;    
            break;
          }
        }
        route.push_back(*all[next]);
        x_vals[k][curr][next]--;
        curr = next;
      } while (curr != depot);
      //get remainig neighboring in AFSs
      //use queue here
      while (!routeAfss.empty()) {
        int afs = routeAfss.front();
        routeAfss.pop();
        //while has path
        while (true) {
          bool hasPath = false;
          //dfs
          list<Vertex> partial_route;
          curr = afs;
          next = afs;
          partial_route.push_back(*all[curr]);
          while (true) {
            //get new neighborhood
            for (auto it = all.rbegin(); it != all.rend(); ++it) {
              int i = it->first;
              if (x_vals[k][curr][i] > INTEGRALITY_TOL) {
                //check if i is an afs
                if (!customers.count(i) && i != depot)
                  routeAfss.push(i);
                hasPath = true;
                next = i;    
                break;
              }
            }
            x_vals[k][curr][next]--;
            if (next == afs)
              break;
            partial_route.push_back(*all[next]);
            curr = next;
          }
          if (!hasPath)
            break;
          //find pointer
          auto it = route.begin();
          for (; it->id != afs; it++);
          route.splice(it, partial_route);
        }
      }
      if (route.size() > 2)
        routes.push_back(route);
    }
    solution = new Gvrp_solution(routes, instance);
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting routes");
  }
}

void Cubic_model::endVals(){
  for (int k = 0; k < instance.maxRoutes; k++){ 
    for (const pair<int, const Vertex *>& p : all)
      x_vals[k][p.first].end();
    x_vals[k].end();
  }
  x_vals.end();
}

void Cubic_model::endVars(){
  for (int k = 0; k < instance.maxRoutes; k++){ 
    for (const pair<int, const Vertex *>& p : all)
      x[k][p.first].end();
    x[k].end();
  }
  x.end();
  e.end(); 
}
