#include "models/vertex.hpp"
#include "models/distances_enum.hpp"
#include "models/cplex/mip_solution_info.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/cplex/gvrp_model.hpp"
#include "models/gvrp_models/cplex/emh_model/emh_model.hpp"

#include <sstream>
#include <list>
#include <time.h> 
#include <string> 

using namespace models;
using namespace models::cplex;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex;
using namespace models::gvrp_models::cplex::emh_model;

using namespace std;

EMH_model::EMH_model(const Gvrp_instance& instance, unsigned int time_limit) : Gvrp_model(instance, time_limit), depotDummy(new Vertex(instance.depot.id, instance.depot.x, instance.depot.y, instance.afss.front().serviceTime)), RELAXED(false) {
  if (instance.distances_enum != METRIC)
    throw string("Error: The compact model requires a G-VRP instance with metric distances");
  //populating all map and customers set
  all[instance.depot.id] = &instance.depot;
  for (const Vertex& customer: instance.customers) {
    all[customer.id] = &customer;
    customers.insert(customer.id);
  }
  //creating dummies
  for (const Vertex& afs: instance.afss) {
    afs_dummies[afs.id].push_back(afs.id);
    all[afs.id] = &afs;
    dummies[afs.id] = &afs;
  }
  int dummy_id = all.rbegin()->second->id;
  //afs dummies
  for (const Vertex& afs: instance.afss)
    for (int i = 0; i < instance.customers.size(); ++i) {
      afs_dummies[afs.id].push_back(++dummy_id);
      all[dummy_id] = &afs;
      dummies[dummy_id] = &afs;
    }
  //depot dummies
  for (int i = 0; i < instance.customers.size() + 1; ++i) {
    afs_dummies[instance.depot.id].push_back(++dummy_id);
    all[dummy_id] = depotDummy;
    dummies[dummy_id] = depotDummy;
  }
} 

EMH_model::~EMH_model() {
  delete depotDummy;
}

pair<Gvrp_solution, Mip_solution_info> EMH_model::run(){
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
    return make_pair(*solution, mipSolInfo);
  } catch (IloException& e) {
    output_exception<<"Concert exception caught: " << e<<endl;
    throw output_exception.str();
  } catch (string s) {
    throw s;
  }
}

void EMH_model::createVariables(){
  x = Matrix2DVar (env, all.size());
  e = IloNumVarArray (env, all.size(), 0, instance.vehicleFuelCapacity, IloNumVar::Float);
  t = IloNumVarArray (env, all.size(), 0, instance.timeLimit, IloNumVar::Float);
  try {
    //setting names
    stringstream nameStream;
    for (const pair<int, const Vertex *>& p : all){
      int i = p.first;
      //e var
      nameStream<<"e["<<i<<"]";
      e[i].setName(nameStream.str().c_str());
      nameStream.clear();
      nameStream.str("");
      //t var
      nameStream<<"t["<<i<<"]";
      t[i].setName(nameStream.str().c_str());
      nameStream.clear();
      nameStream.str("");
      //x var
      x[i] = IloNumVarArray(env, all.size(), 0, 1, RELAXED ? IloNumVar::Float : IloNumVar::Int);
      for (const pair<int, const Vertex *>& p1 : all){
        int j = p1.first;
        nameStream<<"x["<<i<<"]["<<j<<"]";
        x[i][j].setName(nameStream.str().c_str());
        nameStream.clear();
        nameStream.str("");
      }
    }
  }catch (IloException& e) {
    throw e;
  } catch(...){
    throw string("Error in creating variables");
  }
}

void EMH_model::createObjectiveFunction() {
  //objective function
  try{
    IloExpr fo (env);
    for (const pair<int, const Vertex *>& p : all){
      int i = p.first;
      for (const pair<int, const Vertex *>& p1 : all){
        int j = p1.first;
        fo +=  instance.distances[p.second->id][p1.second->id] * x[i][j];
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

void EMH_model::createModel() {
  try {
    //setup
    int depot = instance.depot.id;
    double beta = instance.vehicleFuelCapacity;
    double T = instance.timeLimit;
    //constraints
    IloExpr expr(env),
            expr1(env);    
    IloConstraint c;
    stringstream constraintName;
    //x_{ii} = 0, \forall v_i \in V'
    for (const pair<int, const Vertex *>& p : all) {
      int i = p.first;
      model.add(x[i][i] == 0);
    }
    //\sum_{v_j \in V' : v_i \neq v_j} x_{ij} = 1, \forall v_i \in C
    for (const Vertex& customer : instance.customers){
      int i = customer.id;
      for (const pair<int, const Vertex *>& p1 : all) {
        int j = p1.first;
        if (i != j)
          expr += x[i][j];
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
    //\sum_{v_j \in V' : v_f \neq v_j} x_{fj} \leqslant 1, \forall v_f inF_0 
    for (const pair<int, const Vertex *>& p : dummies) {
      int f = p.first;
      for (const pair<int, const Vertex *>& p1 : all) {
        int j = p1.first;
        if (f != j)
          expr += x[f][j];
      }
      c = IloConstraint (expr <= 1);
      constraintName<<"dummy "<<f<<" must be used at most once";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{v_j \in V' : v_j \neq v_i} x_{ij} = \sum_{v_j \in V' : v_j \neq v_i} x_{ji}, \forall v_i \in V'
    for (const pair<int, const Vertex *>& p : all){
      int i = p.first;
      for (const pair<int, const Vertex *>& p1 : all){
        int j = p1.first;
        if (i != j) {
          expr += x[i][j];
          expr1 += x[j][i]; 
        }
      }
      c = IloConstraint (expr == expr1);
      constraintName<<i<<" # entering edges == "<<i<<" # exiting edges";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr1.end();
      expr = IloExpr(env);
      expr1 = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }    
    //\sum_{v_j \in V' : v_j \neq v_0} x_{0j} \leqslant m
    for (const pair<int, const Vertex *>& p : all)
      expr += x[depot][p.first];
    c = IloConstraint (expr <= instance.maxRoutes);
    c.setName(string(" # routes limit == ").c_str());
    model.add(c);
    expr.end();
    expr = IloExpr(env);
    //t_j \geq t_i + (t_{ij}  + p_j) * x_{ij} - T (1 - x_{ij}), \forall v_j \in V\backslash\{v_0\},\forall v_i \in V
    for (const pair<int, const Vertex *>& p1 : all) {
      int j = p1.first;
      if (j != depot) {
        for (const pair<int, const Vertex *>& p : all) {
          int i = p.first;
          if (i != j) {
            expr = t[i] + x[i][j] * (instance.time(p.second->id, p1.second->id) + p1.second->serviceTime) - T * (1 -  x[i][j]);
            c = IloConstraint (t[j] >= expr);
            c.setName("updating time");
            model.add(c);
            expr.end();
            expr = IloExpr (env);
          }
        }
      }
    }
    //t_{0j} \leq t_j \leq T - t_{j0}_ , \forall v_j \in V\backslash\{v_0\}
    for (const pair<int, const Vertex *>& p : all) {
      int j = p.first;
      if (j != depot) {
        c = IloConstraint (t[j] >= instance.time(depot, p.second->id));
        c.setName("time variable lb");
        model.add(c);
        c = IloConstraint (t[j] <= T - instance.time(p.second->id, depot));
        c.setName("time variable ub");
        model.add(c);
      }
    }
    //e_f = \beta, \forall v_f \in F_0
    for (const pair<int, const Vertex *>& p : dummies) {
      c = IloConstraint (e[p.first] == beta);
      c.setName("e_f = beta");
      model.add(c);
    }
    c = IloConstraint (e[depot] == beta);
    c.setName("e_depot = beta");
    model.add(c);
    //e_j \leq e_i - c_{ij} x_{ij} + \beta (1 - x_{ij}), \forall v_j \in C,\forall v_i \in V'
    for (const Vertex& customer : instance.customers) {
      int j = customer.id;
      for (const pair<int, const Vertex *>& p : all) {
        int i =  p.first;
        if (i != j) {
          expr = e[i] - x[i][j] * instance.fuel(p.second->id, j) + beta * (1 -  x[i][j]);
          c = IloConstraint (e[j] <= expr);
          c.setName("updating fuel level");
          model.add(c);
          expr.end();
          expr = IloExpr (env);
        }
      }
    }
    //e_i \geq c_{ij} x_{ij}, \forall v_i, \forall v_j \in V'
    for (const pair<int, const Vertex *>& p : all) {
      int i = p.first;
      for (const pair<int, const Vertex *>& p1 : all) {
        int j = p1.first;
        expr = instance.fuel(p.second->id, p1.second->id) * x[i][j];
        c = IloConstraint (e[i] >= expr);
        c.setName("disabling infeasible edges");
        model.add(c);
        expr.end();
        expr = IloExpr(env);
      }
    }
    cplex = IloCplex(model);
    //extra steps
    extraStepsAfterModelCreation();
  } catch (IloException& e) {
    throw e;
  } catch (string s) {
    throw s;
  }
}

void EMH_model::extraStepsAfterModelCreation() {
  //
}

void EMH_model::setCustomParameters(){
  try{
    setParameters();
    cplex.setParam(IloCplex::Param::Simplex::Tolerances::Optimality, 1e-3);
    //for the user cut callback
    cplex.setParam(IloCplex::Param::Preprocessing::Linear, 0);
    //for the lazy constraint callback, although this formulation does not make use of lazy constraints, (this parameter is being defined to standarize the experiments (since the cubic formulations makes use of lazy constraints)
    cplex.setParam(IloCplex::Param::Preprocessing::Reduce, 2);
    //this parameter is being defined to standarize the experiments (since the cubic formulations makes use of lazy constraints)
    cplex.setParam(IloCplex::Param::Threads, 1);
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in setting parameters");
  }
}

void EMH_model::fillX_vals(){
  //getresult
  try{
    x_vals = Matrix2DVal (env, all.size());
    for (const pair<int, const Vertex *>& p : all){
      int i = p.first;
      x_vals[i] = IloNumArray (env, all.size(), 0, 1, IloNumVar::Int);
      cplex.getValues(x_vals[i], x[i]);
    }
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting solution");
  }
}

void EMH_model::createGvrp_solution(){
  try{
    list<list<Vertex> > routes;
    list<Vertex> route;
    int depot = instance.depot.id;
    int curr,
        next;    
    //checking the depot neighboring
    for (const pair<int, const Vertex *>& p : all){
      //dfs
      curr = depot;
      next = p.first; 
      if (x_vals[curr][next] > INTEGRALITY_TOL && curr != next) {
        route.push_back(Vertex(*all[curr]));
        for (; next != depot; route.push_back(Vertex(*all[curr]))) {
          curr = next;
          for (const pair<int, const Vertex *>& p1 : all) {
            int j = p1.first;
            if (x_vals[curr][j] > INTEGRALITY_TOL) {
              next = j;
              break;
            }
          }
        }
        route.push_back(Vertex(*all[next]));
        routes.push_back(route);
        route = list<Vertex> ();
      }
    }
    solution = new Gvrp_solution(routes, instance);
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting routes");
  }
}

void EMH_model::endVals(){
  for (const pair<int, const Vertex *>& p : all) 
    x[p.first].end();
  x.end();
}

void EMH_model::endVars(){
  for (const pair<int, const Vertex *>& p : all)
    x[p.first].end();
  x.end();
  e.end(); 
  t.end(); 
}
