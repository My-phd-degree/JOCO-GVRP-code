#include "utils/util.hpp"
#include "models/vertex.hpp"
#include "models/objective_function_enum.hpp" 
#include "models/distances_enum.hpp"
#include "models/cplex/mip_solution_info.hpp"
#include "models/cplex/depth_node_callback.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/cplex/gvrp_model.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/afs_bounds_consec.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/preprocessing.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/user_constraint.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/lazy_constraint.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/subcycle_user_constraint.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/invalid_edge_preprocessing.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/invalid_edge_preprocessing_2.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/invalid_edge_preprocessing_3.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/invalid_edge_preprocessing_4.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"

#include <sstream>
#include <list>
#include <float.h>
#include <time.h> 
#include <string> 
#include <unordered_set>
#include <iterator>

using namespace utils;
using namespace models;
using namespace models::cplex;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex;
using namespace models::gvrp_models::cplex::afs_bounds_consec;

using namespace std;

Afs_bounds_consec::Afs_bounds_consec(const Gvrp_instance& instance, unsigned int time_limit, const Vertex& afs_, Objective_function_enum objective_function_enum_) : Gvrp_model(instance, time_limit), afs(afs_), c0(vector<const Vertex *> (instance.customers.size() + 1)), nGreedyLP(0), BPPTimeLimit(100000000), levelSubcycleCallback(0), nPreprocessings1(0), nPreprocessings2(0), nPreprocessings3(0), nPreprocessings4(0), nImprovedMSTNRoutesLB(0), nBPPNRoutesLB(0), RELAXED(false), objective_function_enum(objective_function_enum_) {
  if (instance.distances_enum != METRIC)
    throw string("Error: The compact model requires a G-VRP instance with metric distances");
  bool found = false;
  for (const Vertex& afs_ : instance.afss)
    if (afs_.id == afs.id) {
      found = true;
      break;
    }
  if (!found)
    throw string("Error: The afs object does not belong to the G-VRP instance");
  //c_0
  c0[0] = &instance.depot;
  customersC0Indexes[instance.depot.id] = 0;
  int i = 0;
  for (const Vertex& customer : instance.customers) {
    c0[i + 1] = &customer;
    customersC0Indexes[customer.id] = i + 1;
    ++i;
  }
  //customer min required fuel
  customersMinRequiredFuel = vector<double> (c0.size() - 1);
  for (int i = 1; i < c0.size(); ++i)
    customersMinRequiredFuel[i - 1] = calculateCustomerMinRequiredFuel(instance, *gvrp_afs_tree, *c0[i]);
  //customer min required time 
  customersMinRequiredTime = vector<double> (c0.size());
  for (int i = 1; i < c0.size(); ++i)
    customersMinRequiredTime[i] = calculateCustomerMinRequiredTime(instance, *gvrp_afs_tree, *c0[i]);
  customersMinRequiredTime[0] = 0.0;
  //_f
  int f = 0;
  for (int r = 1; r < gvrp_afs_tree->f0.size(); ++r) {
    const Vertex * afs = gvrp_afs_tree->f0[r];
    bool valid = false;
    for (int i = 1; i < c0.size(); ++i) 
      for (int r_ = 1; r_ < gvrp_afs_tree->f0.size(); ++r_) {
        int r_id = gvrp_afs_tree->f0[r]->id;
        if (instance.fuel(afs->id, c0[i]->id) + instance.fuel(c0[i]->id, r_id) <= instance.vehicleFuelCapacity 
            && gvrp_afs_tree->times[r] + instance.time(afs->id, c0[i]->id) + c0[i]->serviceTime + instance.time(c0[i]->id, r_id) + gvrp_afs_tree->times[r_] <= instance.timeLimit) {
          valid = true;
          i = c0.size();
          break;
        }
      }
    if (valid) {
      _f.push_back(afs); 
      afss_FIndexes[afs->id] = f;
      ++f;
    }
  }
  //reductions
  const auto& gvrpReducedGraphs = calculateGVRPReducedGraphs (instance, *gvrp_afs_tree);
  gvrpReducedGraphDistances = gvrpReducedGraphs.first;
  gvrpReducedGraphTimes = gvrpReducedGraphs.second;
  //set sol lb
  const auto& closestsDistances = calculateClosestsGVRPCustomers(gvrpReducedGraphDistances, c0);
  solLB = max(calculateGvrpLBByImprovedMST(c0, closestsDistances, gvrpReducedGraphDistances), calculateGvrpLB1(closestsDistances));
  //set n routes lb
  const auto& closestsTimes = calculateClosestsGVRPCustomers(gvrpReducedGraphTimes, c0);
  nRoutesLB = max(int(ceil(calculateGvrpLBByImprovedMSTTime(c0, closestsTimes, gvrpReducedGraphTimes)/instance.timeLimit)), calculateGVRP_BPP_NRoutesLB(instance, c0, closestsTimes, 1000000));
  //user constraints
  user_constraints.push_back(new Subcycle_user_constraint(*this));
  //preprocessings
  preprocessings.push_back(new Invalid_edge_preprocessing(*this));
  preprocessings.push_back(new Invalid_edge_preprocessing_2(*this));
  preprocessings.push_back(new Invalid_edge_preprocessing_3(*this));
  preprocessings.push_back(new Invalid_edge_preprocessing_4(*this));
} 

Afs_bounds_consec::~Afs_bounds_consec() {
  for (Preprocessing * preprocessing : preprocessings)
    delete preprocessing;  
  for (User_constraint * user_constraint : user_constraints)
    delete user_constraint;  
  for (Lazy_constraint * lazy_constraint : lazy_constraints)
    delete lazy_constraint;  
  for (Extra_constraint * extra_constraint : extra_constraints)
    delete extra_constraint;  
  for (Heuristic_callback * heuristic_callback : heuristic_callbacks)
    delete heuristic_callback;  
}

double Afs_bounds_consec::distance (int i, int f, int r, int j) {
  int f_, r_;
  for (int i = 0; i < gvrp_afs_tree->f0.size(); ++i) {
    if (gvrp_afs_tree->f0[i]->id == _f[f]->id)
      f_ = i;
    if (gvrp_afs_tree->f0[i]->id == _f[r]->id)
      r_ = i;
  }
  return instance.distances[c0[i]->id][_f[f]->id] + gvrp_afs_tree->pairCosts[f_][r_] + instance.distances[_f[r]->id][c0[j]->id];
}

double Afs_bounds_consec::time (int i, int f, int r, int j) {
  int f_, r_;
  for (int k = 0; k < gvrp_afs_tree->f0.size(); ++k) {
    if (gvrp_afs_tree->f0[k]->id == _f[f]->id)
      f_ = k;
    if (gvrp_afs_tree->f0[k]->id == _f[r]->id)
      r_ = k;
  }
  return c0[i]->serviceTime + instance.time(c0[i]->id, _f[f]->id) + gvrp_afs_tree->pairTimes[f_][r_] + instance.time(_f[r]->id, c0[j]->id);
}

double Afs_bounds_consec::time(int i, int j) {
  return c0[i]->serviceTime + instance.time(c0[i]->id, c0[j]->id);
}

double Afs_bounds_consec::customersFuel(int i, int j) {
  return instance.fuel(c0[i]->id, c0[j]->id);
}

double Afs_bounds_consec::afsToCustomerFuel(int f, int i) {
  /*
  int f_;
  for (int i = 0; i < gvrp_afs_tree->f0.size(); ++i) 
    if (gvrp_afs_tree->f0[i]->id == _f[f]->id) {
      f_ = i;
      break;
    }
  return instance.fuel(_f[f_]->id, c0[i]->id);
  */
  return instance.fuel(_f[f]->id, c0[i]->id);
}

double Afs_bounds_consec::customerToAfsFuel(int i, int f) {
  /*
  int f_;
  for (int i = 0; i < gvrp_afs_tree->f0.size(); ++i) 
    if (gvrp_afs_tree->f0[i]->id == _f[f]->id) {
      f_ = i;
      break;
    }
  return instance.fuel(c0[i]->id, _f[f_]->id);
    */
  return instance.fuel(c0[i]->id, _f[f]->id);
}

list<Vertex> Afs_bounds_consec::getAFSsShortestPath (const Vertex& ori, const Vertex& dest) {
  //get vertexes
  int f, r;
  for (int i = 0; i < gvrp_afs_tree->f0.size(); ++i) {
    if (gvrp_afs_tree->f0[i]->id == ori.id)
      f = i;
    if (gvrp_afs_tree->f0[i]->id == dest.id)
      r = i;
  }
  //dfs
  list<Vertex> path;
  for (int _r = f; _r != r; path.push_back(Vertex(*gvrp_afs_tree->f0[_r])), _r = gvrp_afs_tree->pairPreds[r][_r]);
  path.push_back(Vertex(*gvrp_afs_tree->f0[r]));
  return path;
}

pair<Gvrp_solution, Mip_solution_info> Afs_bounds_consec::run(){
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
//    cout<<"Setting parameter"<<endl;
    setCustomParameters();
//    cout<<"Solving model"<<endl;
    struct timespec start, finish;
    double elapsed;
    clock_gettime(CLOCK_MONOTONIC, &start);
    if ( !cplex.solve() ) {
//      env.error() << "Failed to optimize LP." << endl;
//      cplex.exportModel("cplexcpp.lp");
      mipSolInfo = Mip_solution_info(-1, cplex.getStatus(), -1, -1);
      endVars();
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
    fillVals();
//    cout<<"Creating GVRP solution"<<endl;
    createGvrp_solution();
    endVals ();
    endVars();
    return make_pair(*solution, mipSolInfo);
  } catch (IloException& e) {
    output_exception<<"Concert exception caught: " << e<<endl;
    throw output_exception.str();
  } catch (string s) {
    throw s;
  }
}

void Afs_bounds_consec::createVariables(){
  y = Matrix4DVar (env, c0.size());
  x = Matrix2DVar (env, c0.size());
  a = Matrix2DVar (env, c0.size() - 1);
  u = Matrix2DVar (env, c0.size());
  v = Matrix2DVar (env, c0.size() - 1);
  try {
    //setting names
    stringstream nameStream;
    for (int i = 0; i < c0.size(); ++i) {
      //x, u, c, and w vars
      x[i] = IloNumVarArray (env, c0.size(), 0, 1, RELAXED ? IloNumVar::Float : IloNumVar::Int);
      u[i] = IloNumVarArray (env, c0.size(), 0, instance.timeLimit, IloNumVar::Float);
      //v and a
      if (i > 0) {
        a[i - 1] = IloNumVarArray (env, c0.size() - 1, 0, instance.vehicleFuelCapacity, IloNumVar::Float);
        v[i - 1] = IloNumVarArray (env, _f.size() + 1, 0, instance.vehicleFuelCapacity, IloNumVar::Float);
        nameStream<<"v["<<i - 1<<"]["<<0<<"]=edge("<<c0[i]->id<<","<<instance.depot.id<<")";
        v[i - 1][0].setName(nameStream.str().c_str());
        nameStream.clear();
        nameStream.str("");
        for (int f = 0; f < _f.size(); ++f) {
          nameStream<<"v["<<i - 1<<"]["<<f + 1<<"]=edge("<<c0[i]->id<<","<<_f[f]->id<<")";
          v[i - 1][f + 1].setName(nameStream.str().c_str());
          nameStream.clear();
          nameStream.str("");
        }
      }
      for (int j = 0; j < c0.size(); ++j) {
        //x
        nameStream<<"x["<<i<<"]["<<j<<"]=edge("<<c0[i]->id<<","<<c0[j]->id<<")";
        x[i][j].setName(nameStream.str().c_str());
        nameStream.clear();
        nameStream.str("");
        //u
        nameStream<<"u["<<i<<"]["<<j<<"]=edge("<<c0[i]->id<<","<<c0[j]->id<<")";
        u[i][j].setName(nameStream.str().c_str());
        nameStream.clear();
        nameStream.str("");
        //a
        if (j > 0 && i > 0) {
          nameStream<<"a["<<i - 1<<"]["<<j - 1<<"]=edge("<<c0[i]->id<<","<<c0[j]->id<<")";
          a[i - 1][j - 1].setName(nameStream.str().c_str());
          nameStream.clear();
          nameStream.str("");
        } 
      }
      //y var
      y[i] = Matrix3DVar (env, _f.size());
      for (int f = 0; f < _f.size(); ++f) {
        y[i][f] = Matrix2DVar(env, _f.size());
        for (int r = 0; r < _f.size(); ++r) {
          y[i][f][r] = IloNumVarArray(env, c0.size(), 0, 1, RELAXED ? IloNumVar::Float : IloNumVar::Int);
          for (int j = 0; j < c0.size(); ++j) {
            nameStream<<"y["<<i<<"]["<<f<<"]["<<r<<"]["<<j<<"]=path("<<c0[i]->id<<", "<<_f[f]->id<<", "<<_f[r]->id<<", "<<c0[j]->id<<")";
            y[i][f][r][j].setName(nameStream.str().c_str());
            nameStream.clear();
            nameStream.str("");
          }
        }
      }
    }
  }catch (IloException& e) {
    throw e;
  } catch(...){
    throw string("Error in creating variables");
  }
}

void Afs_bounds_consec::createObjectiveFunction() {
  //objective function
  try{
    IloExpr fo (env);
    for (int f = 0; f < _f.size(); ++f)
      for (int r = 0; r < _f.size(); ++r) {
        list<Vertex> path = getAFSsShortestPath(*_f[f], *_f[r]);
        for (const Vertex& afs_ : path)
          if (afs_.id == afs.id) {
            for (int i = 0; i < c0.size(); ++i) 
              for (int j = 0; j < c0.size(); ++j) 
                fo += y[i][f][r][j];
            break;
          }
      }
    model = IloModel (env);
    if (objective_function_enum == MIN)
      model.add(IloMinimize(env, fo));
    else if (objective_function_enum == MAX)
      model.add(IloMaximize(env, fo));
  } catch (IloException& e) {
    throw e;
  } catch(...){
    throw string("Error in creating the objective function");
  }
}

void Afs_bounds_consec::createModel() {
  try {
    //preprocessing conditions
    for (Preprocessing* preprocessing : preprocessings) {
      preprocessing->add();
    }
    //constraints
    IloExpr expr(env),
            expr1(env);    
    IloConstraint constraint;
    stringstream constraintName;
    //x_{ii} = 0, \forall v_i \in C_0
    for (int i = 0; i < c0.size(); ++i) 
      model.add(x[i][i] == 0);
    //y_{ifri} = 0, \forall v_i \in C_0, \forall v_f, v_r \in F^{''}
    for (int i = 0; i < c0.size(); ++i) 
      for (int f = 0; f < _f.size(); ++f)
        for (int r = 0; r < _f.size(); ++r)
        model.add(y[i][f][r][i] == 0);
    //\sum_{v_j \in C_0} (x_{ij} + \sum_{v_f, v_r \in F^{''}} y_{ifrj}) = 1, \forall v_i \in C
    for (int i = 1; i < c0.size(); ++i) {
      for (int j = 0; j < c0.size(); ++j) {
        expr += x[i][j];
        for (int f = 0; f < _f.size(); ++f)
          for (int r = 0; r < _f.size(); ++r)
          expr += y[i][f][r][j];
      }
      constraint = IloConstraint (expr == 1);
      constraintName<<"# of exiting edges in customer "<<c0[i]->id<<" must exactly one";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{v_j \in C_0} (x_{ji} + \sum_{v_f, v_r \in F^{''}} y_{jfri}) = 1, \forall v_i \in C
    for (int i = 1; i < c0.size(); ++i) {
      for (int j = 0; j < c0.size(); ++j) {
        expr += x[j][i];
        for (int f = 0; f < _f.size(); ++f)
          for (int r = 0; r < _f.size(); ++r)
          expr += y[j][f][r][i];
      }
      constraint = IloConstraint (expr == 1);
      constraintName<<"# of entering edges in customer "<<c0[i]->id<<" must exactly one";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{v_j \in C_0} (x_{0j} + \sum_{v_f, v_r \in F^{''}} y_{0frj}) \leqslant m 
    for (int j = 0; j < c0.size(); ++j) {
      expr += x[0][j];
      for (int f = 0; f < _f.size(); ++f)
        for (int r = 0; r < _f.size(); ++r)
        expr += y[0][f][r][j];
    }
    constraint = IloConstraint (expr <= instance.maxRoutes);
    constraintName<<"at most "<<instance.maxRoutes<<" routes must be used";
    constraint.setName(constraintName.str().c_str());
    model.add(constraint);
    expr.end();
    expr = IloExpr(env);
    constraintName.clear();
    constraintName.str("");
    //time constraints
    //\sum_{v_i \in C \cup \{v_0\}} u_{ji} = \sum_{v_i \in C \cup \{v_0\}} u_{ij} + \sum_{v_i \in C \cup \{v_0\}} t_{ij} x_{ij} + \sum_{v_i \in C \cup \{v_0\}} \sum_{v_f, v_r \in F^{''}} t_{ifrj} x_{ifrj}, \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      for (int i = 0; i < c0.size(); ++i) {
        expr += u[j][i] - u[i][j] - time(i, j) * x[i][j];
        for (int f = 0; f < _f.size(); ++f)
          for (int r = 0; r < _f.size(); ++r)
            expr -= time(i, f, r, j) * y[i][f][r][j];
      }
      constraint = IloConstraint (expr == 0);
      constraintName<<"time in customer "<<c0[j]->id;
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //t_{0j}(x_{j0} + \sum_{v_f, v_r \in F} t_{jfr0}) \leqslant u_{j0}, \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      expr = x[j][0];
      for (int f = 0; f < _f.size(); ++f)
        for (int r = 0; r < _f.size(); ++r)
          expr += y[j][f][r][0];
      constraint =IloConstraint (customersMinRequiredTime[j] * expr <= u[j][0]);
//      constraint = IloConstraint (time(0, j) * expr <= u[j][0]);
      constraintName<<"lb time in customer "<<c0[j]->id<<" to depot";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //u_{j0} \leqslant (T - t_{j0}) x_{j0} + \sum_{v_f, v_r \in F} (T - t_{jfr0}) y_{jfr0}, \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      expr = (instance.timeLimit - (c0[j]->serviceTime + customersMinRequiredTime[j])) * x[j][0];
 //     expr = (instance.timeLimit - time(j, 0)) * x[j][0];
      for (int f = 0; f < _f.size(); ++f)
        for (int r = 0; r < _f.size(); ++r)
          expr += (instance.timeLimit - time(j, f, r, 0)) * y[j][f][r][0];
      constraint = IloConstraint (expr >= u[j][0]);
      constraintName<<"ub time in customer "<<c0[j]->id<<" to depot";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //u_{0j} = 0, \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      constraint = IloConstraint (u[0][j] == 0);
      constraintName<<"time in depot to "<<c0[j]->id<<" must be 0";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      constraintName.clear();
      constraintName.str("");
    }
    //u_{ij} \geqslant max(t_{0j} - t_{ij}, t_{0i}) x_{ij} + \sum_{v_f, v_r \in F^{''}} max(t_{0j} - t_{ifrj}, t_{0i}) y_{ifrj}, \forall v_i, v_j \in C
    for (int i = 1; i < c0.size(); ++i) 
      for (int j = 1; j < c0.size(); ++j) {
//        expr = max(time(0, j) - time(i, j), time(0, i)) * x[i][j];
        expr = max(customersMinRequiredTime[j] - time(i, j), customersMinRequiredTime[i]) * x[i][j];
        for (int f = 0; f < _f.size(); ++f)
          for (int r = 0; r < _f.size(); ++r)
//        expr += max(time(0, j) - time(i, f, j), time(0, i)) * y[i][f][j];
            expr += max(customersMinRequiredTime[j] - time(i, f, r, j), customersMinRequiredTime[i]) * y[i][f][r][j];
        constraint = IloConstraint (u[i][j] >= expr);
        constraintName<<"lb time in "<<c0[i]->id<<" to "<<c0[j]->id;
        constraint.setName(constraintName.str().c_str());
        model.add(constraint);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    //u_{ij} \leqslant max(T - t_{j0} - t_{ij}, T - t_{i0}) x_{ij} + \sum_{v_f, v_r \in F^{''}} max(T - t_{j0} - t_{ifrj}, T - t_{i0}) y_{ifrj}, \forall v_i, v_j \in C
    for (int i = 1; i < c0.size(); ++i) 
      for (int j = 1; j < c0.size(); ++j) {
 //       expr = min(instance.timeLimit - time(j, 0) - time(i, j), instance.timeLimit - time(i, 0)) * x[i][j];
        expr = min(instance.timeLimit - (c0[j]->serviceTime + customersMinRequiredTime[j]) - time(i, j), instance.timeLimit - (c0[i]->serviceTime + customersMinRequiredTime[i])) * x[i][j];
        for (int f = 0; f < _f.size(); ++f)
          for (int r = 0; r < _f.size(); ++r)
//            expr += min(instance.timeLimit - time(j, 0) - time(i, f, r, j), instance.timeLimit - time(i, 0)) * y[i][f][r][j];
            expr += min(instance.timeLimit - (c0[j]->serviceTime + customersMinRequiredTime[j]) - time(i, f, r, j), instance.timeLimit - (c0[i]->serviceTime + customersMinRequiredTime[i])) * y[i][f][r][j];
        constraint = IloConstraint (u[i][j] <= expr);
        constraintName<<"lb time in "<<c0[i]->id<<" to "<<c0[j]->id;
        constraint.setName(constraintName.str().c_str());
        model.add(constraint);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    //energy constraints
    //\sum_{v_i \in C} a_{i, j} - a_{j, i} = \sum_{v_f \in F^{''}} v_{jf} + \sum_{v_i \in C} e_{ji} x_{ji} - \sum_{v_i \in C_0} \sum_{v_f, v_r \in F^{''}} (\beta - e_{rj}) y_{ifrj} - (\beta - e_{0j}) x_{0j}, \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      for (int f = 0; f < _f.size() + 1; ++f)
        expr -= v[j - 1][f];
      for (int i = 0; i < c0.size(); ++i) {
        if (i > 0) 
          expr += a[i - 1][j - 1] - a[j - 1][i - 1] - customersFuel(j, i) * x[j][i];
        for (int f = 0; f < _f.size(); ++f)
          for (int r = 0; r < _f.size(); ++r)
            expr += (instance.vehicleFuelCapacity - afsToCustomerFuel(r, j)) * y[i][f][r][j];
      }
      expr += (instance.vehicleFuelCapacity - customersFuel(0, j)) * x[0][j];
      constraint = IloConstraint (expr == 0);
      constraintName<<"customer "<<c0[j]->id<<" energy update";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //a_{ij} >= x_{ij} * max (min_{v_f \in F_0} e_{jf}, min_{v_f \in F_0} e_{if} - e_{ij}), \forall v_i, v_j \in C
    for (int j = 1; j < c0.size(); ++j) 
      for (int i = 1; i < c0.size(); ++i) {
        /*
        //min_{v_f \in F_0} e_{jf} and min_{v_f \in F_0} e_{if}
        double closestAfsToJ = customerToAfsFuel(j, 0),
               closestAfsToI = customerToAfsFuel(i, 0);
        for (int f = 1; f < _f.size(); ++f) {
          closestAfsToJ = min(closestAfsToJ, customerToAfsFuel(j, f));
          closestAfsToI = min(closestAfsToI, customerToAfsFuel(i, f));
        }
        constraint = IloConstraint (a[i - 1][j - 1] >= x[i][j] * max(closestAfsToJ, closestAfsToI - customersFuel(i, j)));
        */
        constraint = IloConstraint (a[i - 1][j - 1] >= x[i][j] * max(customersMinRequiredFuel[j - 1], customersMinRequiredFuel[i - 1] - customersFuel(i, j)));
        constraintName<<"a["<<i - 1<<"]["<<j - 1<<"] lb";
        constraint.setName(constraintName.str().c_str());
        model.add(constraint);
        constraintName.clear();
        constraintName.str("");
      }
    //a_{ij} <= x_{ij} * min (\beta - min_{v_f \in F_0} e_{fj}, \beta - min_{v_f \in F_0} e_{fi} - e_{ij}), \forall v_i, v_j \in C
    for (int j = 1; j < c0.size(); ++j) 
      for (int i = 1; i < c0.size(); ++i) {
//        //min_{v_f \in F_0} e_{fj} and min_{v_f \in F_0} e_{fi}
//       double closestAfsToJ = afsToCustomerFuel(0, j),
//              closestAfsToI = afsToCustomerFuel(0, i);
//       for (int f = 1; f < _f.size(); ++f) {
//         closestAfsToJ = min(closestAfsToJ, afsToCustomerFuel(f, j));
//         closestAfsToI = min(closestAfsToI, afsToCustomerFuel(f, i));
//       }
//       constraint = IloConstraint (a[i - 1][j - 1] <= x[i][j] * min(instance.vehicleFuelCapacity - closestAfsToI - customersFuel(i, j), instance.vehicleFuelCapacity - closestAfsToJ));
        constraint = IloConstraint (a[i - 1][j - 1] <= x[i][j] * min(instance.vehicleFuelCapacity - customersMinRequiredFuel[i - 1] - customersFuel(i, j), instance.vehicleFuelCapacity - customersMinRequiredFuel[j - 1]));
        constraintName<<"a["<<i - 1<<"]["<<j - 1<<"] ub";
        constraint.setName(constraintName.str().c_str());
        model.add(constraint);
        constraintName.clear();
        constraintName.str("");
      }
    //v_{j0} \geqslant x_{j0} * e_{j0}, \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      constraint = IloConstraint (v[j - 1][0] >= x[j][0] * customersFuel(j, 0));
      constraintName<<"v["<<j - 1<<"][0] lb";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      constraintName.clear();
      constraintName.str("");
    }
    //v_{j0} \leqslant x_{j0} * (\beta - min_{v_f \in F_0} e_{fj}), \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      constraint = IloConstraint (v[j - 1][0] <= x[j][0] * (instance.vehicleFuelCapacity - customersMinRequiredFuel[j - 1]));
      constraintName<<"v["<<j - 1<<"][0] ub";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      constraintName.clear();
      constraintName.str("");
    }
    //v_{jf} \geqslant \sum_{v_i \in C_0} \sum_{v_r \in F^{''}} z_{jfri} * e_{jf}, \forall v_j \in C, \forall v_f \in F^{''}
    for (int j = 1; j < c0.size(); ++j) {
      for (int f = 0; f < _f.size(); ++f) {
        for (int r = 0; r < _f.size(); ++r) 
          for (int i = 0; i < c0.size(); ++i) 
            expr += y[j][f][r][i];
        constraint = IloConstraint (v[j - 1][f + 1] >= expr * customerToAfsFuel(j, f));
        constraintName<<"v["<<j - 1<<"]["<<f + 1<<"] lb";
        constraint.setName(constraintName.str().c_str());
        model.add(constraint);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    }
    //v_{jf} \leqslant (\beta - min_{v_f \in F_0} e_{fj}) * \sum_{v_r \in F^{''}} \sum_{v_i \in C_0} z_{jfri}, \forall v_j \in C, \forall v_f \in F
    for (int j = 1; j < c0.size(); ++j) {
      for (int f = 0; f < _f.size(); ++f) {
        for (int r = 0; r < _f.size(); ++r) 
          for (int i = 0; i < c0.size(); ++i) 
            expr += y[j][f][r][i];
        constraint = IloConstraint (v[j - 1][f + 1] <= expr * (instance.vehicleFuelCapacity - customersMinRequiredFuel[j - 1]));
        constraintName<<"v["<<j - 1<<"]["<<f + 1<<"] ub";
        constraint.setName(constraintName.str().c_str());
        model.add(constraint);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    }
    //no 2 subcycles
    //x_{ij} + x_{ji} + \sum_{v_f \in F} z_{jfi} + z_{ifj} \leqslant 1, \forall v_i, v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      for (int i = 1; i < c0.size(); ++i) {
        expr = x[i][j] + x[j][i];
        for (int f = 0; f < _f.size(); ++f) 
          for (int r = 0; r < _f.size(); ++r) 
            expr += y[j][f][r][i] + y[i][f][r][j];
        constraint = IloConstraint (expr <= 1);
        constraintName<<"no 2 subcyle between customers "<<i<<", and "<<j;
        constraint.setName(constraintName.str().c_str());
        model.add(constraint);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    }
    //new inequalities
    //solution lb
    for (int i = 0; i < c0.size(); ++i) 
      for (int j = 0; j < c0.size(); ++j) {
        expr +=  instance.distances[c0[i]->id][c0[j]->id] * x[i][j];
        for (int f = 0; f < _f.size(); ++f)
          for (int r = 0; r < _f.size(); ++r)
            expr += (instance.distances[c0[i]->id][_f[f]->id] + gvrp_afs_tree->pairCosts[f][r] + instance.distances[_f[r]->id][c0[j]->id]) * y[i][f][r][j];
      }
    constraint = IloConstraint (expr >= solLB);
    constraint.setName("solution LB");
    model.add(constraint);
    expr.end();
    expr = IloExpr(env);
    //n routes LB
    for (int i = 0; i < c0.size(); ++i) {
      expr += x[0][i];
      for (int f = 0; f < _f.size(); ++f)
        for (int r = 0; r < _f.size(); ++r)
          expr += y[0][f][r][i];
    }
    constraint = IloConstraint (expr >= nRoutesLB);
    constraint.setName("nRoutes LB");
    model.add(constraint);
    expr.end();
    expr = IloExpr(env);









    /*
    vector<vector<int>> routes_ = {
    {0, 6, 7, 9, 7, 2, 10, 1, 0},
    {0, 10, 12, 3, 8, 10, 4, 11, 15, 10, 0}
    };
    list<list<Vertex>> routes;
    for (const vector<int>& route_ : routes_) {
      list<Vertex> route;
      for (int node : route_) 
        if (customersC0Indexes.count(node))
          route.push_back(Vertex(*c0[customersC0Indexes[node]]));
        else if (afss_FIndexes.count(node))
          route.push_back(Vertex(*_f[afss_FIndexes[node]]));
      routes.push_back(route);
    }
    double currFuel, 
           currTime;
    for (const list<Vertex>& route : routes) {
      currFuel = instance.vehicleFuelCapacity;
      currTime = 0.0;
      list<Vertex>::const_iterator curr = route.begin(), 
        prev = curr;
      for (++curr; curr != route.end(); prev = curr, ++curr) {
        auto currIndex = customersC0Indexes.find(curr->id);
        int i = customersC0Indexes[prev->id];
        //is a customer
        if (currIndex != customersC0Indexes.end()) {
          int j = currIndex->second;
          model.add(x[i][j] == 1);
          cout<<i<<" "<<j<<endl;
        } else {
          //is an afs 
          int f = afss_FIndexes[curr->id];
          int r;
          //get path
          for (currIndex = customersC0Indexes.find(curr->id); currIndex == customersC0Indexes.end(); r = afss_FIndexes[curr->id], ++curr, currIndex = customersC0Indexes.find(curr->id));
          int j = customersC0Indexes[curr->id];
          cout<<i<<" "<<f<<" "<<r<<" "<<j<<endl;
          model.add(y[i][f][r][j] == 1);
        }
      }
    }
    */







    //extra constraints
    for (Extra_constraint* extra_constraint : extra_constraints) 
      extra_constraint->add();
    //init
    cplex = IloCplex(model);
    //lazy constraints
    for (Lazy_constraint* lazy_constraint : lazy_constraints)
      cplex.use(lazy_constraint);
    //user cuts
    for (User_constraint* user_constraint : user_constraints)
      cplex.use(user_constraint);
    //heuristic callback
    for (Heuristic_callback* heuristic_callback : heuristic_callbacks)
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

void Afs_bounds_consec::extraStepsAfterModelCreation() {
  //
}

void Afs_bounds_consec::setCustomParameters(){
  try{
    setParameters();
    //for the user cut callback, although this formulation does not make use of lazy constraints, (this parameter is being defined to standarize the experiments (since the cubic formulations makes use of user constraints)
//    cplex.setParam(IloCplex::Param::Preprocessing::Linear, 0);
    //for the lazy constraint callback, although this formulation does not make use of lazy constraints, (this parameter is being defined to standarize the experiments (since the cubic formulations makes use of lazy constraints)
//    cplex.setParam(IloCplex::Param::Preprocessing::Reduce, 2);
    //this parameter is being defined to standarize the experiments (since the cubic formulations makes use of lazy constraints)
    cplex.setParam(IloCplex::Param::Threads, 1);
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in setting parameters");
  }
}

void Afs_bounds_consec::fillVals(){
  //getresult
  try{
    x_vals = Matrix2DVal (env, c0.size());
    y_vals = Matrix4DVal (env, c0.size());
    for (int i = 0; i < c0.size(); ++i){
      x_vals[i] = IloNumArray (env, c0.size(), 0, 1, IloNumVar::Int);
      y_vals[i] = Matrix3DVal (env, _f.size());
      cplex.getValues(x_vals[i], x[i]);
      for (int f = 0; f < _f.size(); ++f) {
        y_vals[i][f] = Matrix2DVal(env, _f.size());
        for (int r = 0; r < _f.size(); ++r){
          y_vals[i][f][r] = IloNumArray(env, c0.size(), 0, 1, IloNumVar::Int);
          cplex.getValues(y_vals[i][f][r], y[i][f][r]);
        }
      }
    }
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting solution");
  }
}

void Afs_bounds_consec::createGvrp_solution() {
  try {
    /*
    for (int f = 0; f < _f.size(); ++f)
      cout<<_f[f]->id<<" ";
    cout<<endl;
    for (int i = 0; i < c0.size(); ++i) 
      for (int j = 0; j < c0.size(); ++j) {
        if (x_vals[i][j] > INTEGRALITY_TOL) 
          cout<<"("<<c0[i]->id<<", "<<c0[j]->id<<")"<<endl;
        for (int f = 0; f < _f.size(); ++f)
          for (int r = 0; r < _f.size(); ++r)
            if (y_vals[i][f][r][j] > INTEGRALITY_TOL) 
              cout<<"("<<c0[i]->id<<", "<<_f[f]->id<<", "<<_f[r]->id<<", "<<c0[j]->id<<"): "<<time(i, f, r, j)<<endl;
      }

    IloNumArray t_vals (env, c0.size(), 0, instance.timeLimit, IloNumVar::Float);
    cplex.getValues(t_vals, t);
    for (int i = 1; i < c0.size(); ++i)
      cout<<c0[i]->id<<" "<<t_vals[i - 1]<<endl;

    for (int f = 0; f < _f.size(); ++f)
      for (int r = 0; r < _f.size(); ++r) {
        list<Vertex> path = getAFSsShortestPath(*_f[f], *_f[r]);
        double time = path.begin()->serviceTime;
        for (list<Vertex>::const_iterator curr = next(path.begin()); curr != path.end(); ++curr)
          time += instance.time(curr->id, prev(curr)->id) + curr->serviceTime;
        int f__, r__;
        for (int k = 0; k < gvrp_afs_tree->f0.size(); ++k) {
          if (gvrp_afs_tree->f0[k]->id == _f[f]->id)
            f__ = k;
          if (gvrp_afs_tree->f0[k]->id == _f[r]->id)
            r__ = k;
        }
        cout<<gvrp_afs_tree->pairTimes[f__][r__]<<" "<<time<<endl;
      }
      */

    list<list<Vertex>> routes;
    list<Vertex> allNodes;
    int curr = 0;
    for (int i = 0; i < c0.size(); ++i) {
      if (x_vals[curr][i] > INTEGRALITY_TOL) {
        if (allNodes.size() == 0 || allNodes.back().id != c0[curr]->id)
          allNodes.push_back(Vertex(*c0[curr]));
        x_vals[curr][i] = 0;
        curr = i;
        i = -1;
      } else {
        for (int f = 0; f < _f.size(); ++f)
          for (int r = 0; r < _f.size(); ++r)
            if (y_vals[curr][f][r][i] > INTEGRALITY_TOL) {
              if (allNodes.size() == 0 || allNodes.back().id != c0[curr]->id)
                allNodes.push_back(Vertex(*c0[curr]));
              //insert shortest path
              list<Vertex> path = getAFSsShortestPath(*_f[f], *_f[r]);
              allNodes.insert(allNodes.end(), path.begin(), path.end());
              allNodes.push_back(Vertex(*c0[i]));
              y_vals[curr][f][r][i] = 0;
              curr = i;
              i = -1;
              f = _f.size();
              break;
            }
      }
    }
    /*
    for(const Vertex& node : allNodes)
      cout<<node.id<<", ";
    cout<<endl;
    */
    //get routes
    list<Vertex>::iterator beg = allNodes.begin();
    for (list<Vertex>::iterator end = next(beg); end != allNodes.end(); ++end)
      if (end->id == instance.depot.id) {
        list<Vertex> route;
        route.insert(route.end(), beg, next(end));
        routes.push_back(route);
        beg = end;
      }
    list<Vertex> route;
    route.insert(route.end(), beg, allNodes.end());
    route.push_back(instance.depot);
    if (route.size() > 2)
      routes.push_back(route);
    solution = new Gvrp_solution(routes, instance);
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting routes");
  }
}

void Afs_bounds_consec::endVals () {
  //end vals
  for (int i = 0; i < c0.size(); ++i) {
    for (int f = 0; f < _f.size(); ++f) {
      for (int r = 0; r < _f.size(); ++r)
        y_vals[i][f][r].end();
      y_vals[i][f].end();
    }
    y_vals[i].end();
    x_vals[i].end();
  }
  y_vals.end();
  x_vals.end();
}

void Afs_bounds_consec::endVars(){
  //end vals
  for (int i = 0; i < c0.size(); ++i) {
    if (i > 0) {
      a[i - 1].end();
      v[i - 1].end();
    }
    u[i].end();
    for (int f = 0; f < _f.size(); ++f) {
      for (int r = 0; r < _f.size(); ++r)
        y[i][f][r].end();
      y[i][f].end();
    }
    y[i].end();
    x[i].end();
  }
  y.end();
  x.end();
  a.end();
  u.end();
  v.end();
}
