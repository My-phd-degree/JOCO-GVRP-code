#include "utils/util.hpp"
#include "models/vertex.hpp"
#include "models/distances_enum.hpp"
#include "models/cplex/mip_solution_info.hpp"
#include "models/cplex/depth_node_callback.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/cplex/gvrp_model.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/matheus_model_4.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/user_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/lazy_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/subcycle_user_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/greedy_lp_heuristic.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/invalid_edge_preprocessing.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/invalid_edge_preprocessing_2.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/invalid_edge_preprocessing_3.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/invalid_edge_preprocessing_4.hpp"
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
using namespace models::gvrp_models::cplex::matheus_model_4;

using namespace std;

Matheus_model_4::Matheus_model_4(const Gvrp_instance& instance, unsigned int time_limit) : Gvrp_model(instance, time_limit), c0(vector<const Vertex *> (instance.customers.size() + 1)), nGreedyLP(0), BPPTimeLimit(100000000), levelSubcycleCallback(0), nPreprocessings1(0), nPreprocessings2(0), nPreprocessings3(0), nPreprocessings4(0), nImprovedMSTNRoutesLB(0), nBPPNRoutesLB(0), RELAXED(false) {
  if (instance.distances_enum != METRIC)
    throw string("Error: The compact model requires a G-VRP instance with metric distances");
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
  //heuristic callback
//  heuristic_callbacks.push_back(new Greedy_lp_heuristic(*this));
} 

Matheus_model_4::~Matheus_model_4() {
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

double Matheus_model_4::distance (int i, int f, int r, int j) {
  int f_, r_;
  for (int i = 0; i < gvrp_afs_tree->f0.size(); ++i) {
    if (gvrp_afs_tree->f0[i]->id == _f[f]->id)
      f_ = i;
    if (gvrp_afs_tree->f0[i]->id == _f[r]->id)
      r_ = i;
  }
  return instance.distances[c0[i]->id][_f[f]->id] + gvrp_afs_tree->pairCosts[f_][r_] + instance.distances[_f[r]->id][c0[j]->id];
}

double Matheus_model_4::time (int i, int f, int r, int j) {
  int f_, r_;
  for (int k = 0; k < gvrp_afs_tree->f0.size(); ++k) {
    if (gvrp_afs_tree->f0[k]->id == _f[f]->id)
      f_ = k;
    if (gvrp_afs_tree->f0[k]->id == _f[r]->id)
      r_ = k;
  }
  return c0[i]->serviceTime + instance.time(c0[i]->id, _f[f]->id) + gvrp_afs_tree->pairTimes[f_][r_] + instance.time(_f[r]->id, c0[j]->id);
}

double Matheus_model_4::M1(int i, int f, int r, int j) {
  return instance.timeLimit - c0[i]->serviceTime - customersMinRequiredTime[i] - customersMinRequiredTime[j] + time(i, j) + time(i, f, r, j);
}

double Matheus_model_4::M2(int i, int j) {
  return instance.vehicleFuelCapacity - customersMinRequiredFuel[i - 1] - customersMinRequiredFuel[j - 1] + customersFuel(i, j);
}

double Matheus_model_4::time(int i, int j) {
  return c0[i]->serviceTime + instance.time(c0[i]->id, c0[j]->id);
}

double Matheus_model_4::customersFuel(int i, int j) {
  return instance.fuel(c0[i]->id, c0[j]->id);
}

double Matheus_model_4::afsToCustomerFuel(int f, int i) {
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

double Matheus_model_4::customerToAfsFuel(int i, int f) {
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

list<Vertex> Matheus_model_4::getAFSsShortestPath (const Vertex& ori, const Vertex& dest) {
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

pair<Gvrp_solution, Mip_solution_info> Matheus_model_4::run(){
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

void Matheus_model_4::createVariables(){
  y = Matrix4DVar (env, c0.size());
  x = Matrix2DVar (env, c0.size());
  e = IloNumVarArray (env, c0.size() - 1, 0, instance.vehicleFuelCapacity, IloNumVar::Float);
  t = IloNumVarArray (env, c0.size() - 1, 0, instance.timeLimit, IloNumVar::Float);
  try {
    //setting names
    stringstream nameStream;
    for (int i = 0; i < c0.size(); ++i) {
      //x vars
      x[i] = IloNumVarArray (env, c0.size(), 0, 1, RELAXED ? IloNumVar::Float : IloNumVar::Int);
      //e and t vars
      if (i > 0) {
        nameStream<<"e["<<i - 1<<"]=energy in ("<<c0[i]->id<<")";
        e[i - 1].setName(nameStream.str().c_str());
        nameStream.clear();
        nameStream.str("");
        nameStream<<"t["<<i - 1<<"]=time in ("<<c0[i]->id<<")";
        t[i - 1].setName(nameStream.str().c_str());
        nameStream.clear();
        nameStream.str("");
      }
      for (int j = 0; j < c0.size(); ++j) {
        //x
        nameStream<<"x["<<i<<"]["<<j<<"]=edge("<<c0[i]->id<<","<<c0[j]->id<<")";
        x[i][j].setName(nameStream.str().c_str());
        nameStream.clear();
        nameStream.str("");
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

void Matheus_model_4::createObjectiveFunction() {
  //objective function
  try{
    IloExpr fo (env);
    for (int i = 0; i < c0.size(); ++i) 
      for (int j = 0; j < c0.size(); ++j) {
        fo += instance.distances[c0[i]->id][c0[j]->id] * x[i][j];
        for (int f = 0; f < _f.size(); ++f)
          for (int r = 0; r < _f.size(); ++r)
            fo += distance(i, f, r, j) * y[i][f][r][j];
      }
    model = IloModel (env);
    model.add(IloMinimize(env, fo));
  } catch (IloException& e) {
    throw e;
  } catch(...){
    throw string("Error in creating the objective function");
  }
}

void Matheus_model_4::createModel() {
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
    //y_{ifrj} = 0, \forall v_i, v_j \in C, \forall v_f, v_r \in F^{''} : v_0 \in P_{fr}^{G[F_0]}
    for (int f = 0; f < _f.size(); ++f)
      for (int r = 0; r < _f.size(); ++r) {
        list<Vertex> path = getAFSsShortestPath(*_f[f], *_f[r]);
        for (const Vertex& afs : path)
          if (afs.id == instance.depot.id) {
            for (int i = 0; i < c0.size(); ++i) 
              for (int j = 0; j < c0.size(); ++j) 
                if (i != j) 
                  model.add(y[i][f][r][j] == 0);
            break;
          }
      }
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
    //t_{i} - t_{j} + M1_{ij} x_{ij} + (M_{ij} - t_{ij} - t_{ji}) x_{ji} + \sum_{v_f, v_r \in F^{''}} ((M1_{ij} - t_{ij} - t_{ifrj}) y_{ifrj} + (M1_{ij} - t_{ij} - t_{jfri}) y_{jfri}, \forall v_j, v_i \in C: v_i \neq v_j
    for (int i = 1; i < c0.size(); ++i) 
      for (int j = 1; j < c0.size(); ++j) 
        if (i != j) {
          for (int f = 0; f < _f.size(); ++f)
            for (int r = 0; r < _f.size(); ++r) {
              constraint = IloConstraint (t[i - 1] - t[j - 1] + (M1(i, f, r, j) - time(i, f, r, j)) * x[i][j] + (M1(i, f, r, j) - time(i, j) - time(j, i) - time(i, f, r, j)) * x[j][i] + (M1(i, f, r, j) - time(i, j)) * y[i][f][r][j] + (M1(i, f, r, j) - time(i, j) - time(i, f, r, j) - time(j, f, r, i)) * y[j][f][r][i] <= M1(i, f, r, j) - time(i, j) - time(i, f, r, j));
              constraintName<<"time in ("<<c0[i]->id<<", "<<_f[f]->id<<", "<<_f[r]->id<<", "<<c0[j]->id<<")";
              constraint.setName(constraintName.str().c_str());
              model.add(constraint);
              expr.end();
              expr = IloExpr(env);
              constraintName.clear();
              constraintName.str("");
            }
        }
    //x_{0j} t_{0j} + \sum_{v_f, v_r \in F^{''}} y_{0frj} t_{0frj} <= t_{j}, \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      expr += x[0][j] * time(0, j);
      for (int f = 0; f < _f.size(); ++f) 
        for (int r = 0; r < _f.size(); ++r) 
          expr += y[0][f][r][j] * time(0, f, r, j);
      constraint = IloConstraint (expr <= t[j - 1]);
      constraintName<<c0[j]->id<<" time lb";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      constraintName.clear();
      constraintName.str("");
      expr.end();
      expr = IloExpr(env);
    }
    //t_{j} <= T - x_{j0} t_{j0} - \sum_{v_f, v_r \in F^{''}} y_{jfr0} t_{jfr0}, \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      expr += x[j][0] * time(j, 0);
      for (int f = 0; f < _f.size(); ++f) 
        for (int r = 0; r < _f.size(); ++r) 
          expr += y[j][f][r][0] * time(j, f, r, 0);
      constraint = IloConstraint (t[j - 1] <= instance.timeLimit - expr);
      constraintName<<c0[j]->id<<" time ub";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      constraintName.clear();
      constraintName.str("");
      expr.end();
      expr = IloExpr(env);
    }
    //t_{j} <= T - (T - t_{0j}) x_{0j} - \sum_{v_f, v_r \in F^{''}} (T - t_{0frj}) y_{0frj}, \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      expr += (instance.timeLimit - time(0, j)) * x[0][j];
      for (int f = 0; f < _f.size(); ++f) 
        for (int r = 0; r < _f.size(); ++r) 
          expr += (instance.timeLimit - time(0, f, r, j)) * y[0][f][r][j];
      constraint = IloConstraint (t[j - 1] <= instance.timeLimit - expr);
      constraintName<<c0[j]->id<<" time ub 2";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      constraintName.clear();
      constraintName.str("");
      expr.end();
      expr = IloExpr(env);
    }
    //LB_j^T <= t_{j}, \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      constraint = IloConstraint (customersMinRequiredTime[j] <= t[j - 1]);
      constraintName<<c0[j]->id<<" constant time lb";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      constraintName.clear();
      constraintName.str("");
    }
    //t_{j} <= T - LB_j^T, \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      constraint = IloConstraint (t[j - 1] <= instance.timeLimit - customersMinRequiredTime[j]);
      constraintName<<c0[j]->id<<" constant time ub";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      constraintName.clear();
      constraintName.str("");
    }
    //energy constraints
    //e_i - e_j + M2(i, j) x_{ij} + (M2(i, j) - e_{ij} - e_{ji}) x_{ji} <= M2(i, j) - e_{ij}, \forall v_i, v_j \in C: v_i \neq v_j
    for (int j = 1; j < c0.size(); ++j) 
      for (int i = 1; i < c0.size(); ++i) 
        if (i != j) {
          constraint = IloConstraint (e[j - 1] - e[i - 1] + M2(i, j) * x[i][j] + (M2(i, j) - customersFuel(i, j) - customersFuel(j, i)) * x[j][i] <= M2(i, j) - customersFuel(i, j));
          constraintName<<"customers "<<c0[j]->id<<" "<<c0[i]->id<<" energy update";
          constraint.setName(constraintName.str().c_str());
          model.add(constraint);
          expr.end();
          expr = IloExpr(env);
          constraintName.clear();
          constraintName.str("");
        }
    //\sum_{v_j \in C_0} \sum_{v_f, v_r \in F^{''}} y_{ifrj} e_{if} <= e_i, \forall v_i \in C
    for (int i = 1; i < c0.size(); ++i) {
      for (int f = 0; f < _f.size(); ++f) 
        for (int r = 0; r < _f.size(); ++r) 
          for (int j = 0; j < c0.size(); ++j) 
            if (i != j) {
              expr += y[i][f][r][j] * customerToAfsFuel(i, f);
            }
      constraint = IloConstraint (e[i - 1] >= expr + x[i][0] * customersFuel(i, 0));
      constraintName<<"customers "<<c0[i]->id<<" lb";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //e_i <= \beta - \sum_{v_j \in C_0} \sum_{v_f, v_r \in F^{''}} y_{jfri} e_{ri}, \forall v_i \in C
    for (int i = 1; i < c0.size(); ++i) {
      for (int f = 0; f < _f.size(); ++f) 
        for (int r = 0; r < _f.size(); ++r) 
          for (int j = 0; j < c0.size(); ++j) 
            if (i != j) {
              expr1 += y[j][f][r][i] * afsToCustomerFuel(r, i);
            }
      constraint = IloConstraint (e[i - 1] <= instance.vehicleFuelCapacity - expr1 - x[0][i] * customersFuel(0, i));
      constraintName<<"customers "<<c0[i]->id<<" ub";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      expr1.end();
      expr1 = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //LB_i^E <= e_i, \forall v_i \in C
    for (int i = 1; i < c0.size(); ++i) {
      constraint = IloConstraint (customersMinRequiredFuel[i - 1] <= e[i - 1]);
      constraintName<<"customers "<<c0[i]->id<<" constant lb";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      constraintName.clear();
      constraintName.str("");
    }
    //e_i <= \beta - LB_i^E, \forall v_i \in C
    for (int i = 1; i < c0.size(); ++i) {
      constraint = IloConstraint (e[i - 1] <= instance.vehicleFuelCapacity - customersMinRequiredFuel[i - 1]);
      constraintName<<"customers "<<c0[i]->id<<" constant ub";
      constraint.setName(constraintName.str().c_str());
      model.add(constraint);
      constraintName.clear();
      constraintName.str("");
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
      {0, 1, 10, 4, 11, 15, 10, 12, 10, 0, },
      {0, 6, 7, 5, 7, 9, 7, 13, 7, 0  },
      {0, 10, 3, 8, 10, 2, 7, 0},
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
    Gvrp_solution gvrp_sol (routes, instance);
    cout<<gvrp_sol<<endl;
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
          cout<<"("<<c0[i]->id<<", "<<c0[j]->id<<")"<<endl;
          model.add(x[i][j] == 1);
          currFuel -= customersFuel(i, j);
          currTime += time(i, j);
          if (j > 0) {
            model.add(t[j - 1] == currTime);
            model.add(e[j - 1] == currFuel);
          }
        } else {
          //is an afs 
          int f = afss_FIndexes[curr->id];
          int r;
          //get path
          for (currIndex = customersC0Indexes.find(curr->id); currIndex == customersC0Indexes.end(); r = afss_FIndexes[curr->id], ++curr, currIndex = customersC0Indexes.find(curr->id));
          int j = customersC0Indexes[curr->id];
          cout<<"("<<c0[i]->id<<", "<<_f[f]->id<<", "<<_f[r]->id<<", "<<c0[j]->id<<")"<<endl;
          model.add(y[i][f][r][j] == 1);
          currTime += time(i, f, r, j);
          currFuel = instance.vehicleFuelCapacity - afsToCustomerFuel(r, j);
          if (j > 0) {
            model.add(t[j - 1] == currTime);
            model.add(e[j - 1] == currFuel);
          }
        }
      }
    }
    */








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

void Matheus_model_4::extraStepsAfterModelCreation() {
  //
}

void Matheus_model_4::setCustomParameters(){
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

void Matheus_model_4::fillVals(){
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

void Matheus_model_4::createGvrp_solution() {
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

void Matheus_model_4::endVals () {
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

void Matheus_model_4::endVars(){
  //end vals
  for (int i = 0; i < c0.size(); ++i) {
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
  e.end();
  t.end();
}
