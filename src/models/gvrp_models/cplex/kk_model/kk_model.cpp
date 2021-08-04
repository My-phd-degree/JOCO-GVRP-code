#include "models/vertex.hpp"
#include "models/distances_enum.hpp"
#include "models/cplex/mip_solution_info.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/cplex/gvrp_model.hpp"
#include "models/gvrp_models/cplex/kk_model/kk_model.hpp"

#include <sstream>
#include <list>
#include <time.h> 
#include <string> 

using namespace models;
using namespace models::cplex;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex;
using namespace models::gvrp_models::cplex::kk_model;

using namespace std;

KK_model::KK_model(const Gvrp_instance& instance, unsigned int time_limit) : Gvrp_model(instance, time_limit), RELAXED(false) {
  if (instance.distances_enum != METRIC)
    throw string("Error: The compact model requires a G-VRP instance with metric distances");
  //gvrp afs tree
  gvrp_afs_tree = new Gvrp_afs_tree(instance);
  //c_0
  c0 = vector<const Vertex *> (instance.customers.size() + 1);
  c0[0] = &instance.depot;
  int i = 0;
  for (const Vertex& customer : instance.customers) {
    c0[i + 1] = &customer;
    customersC0Indexes[customer.id] = i + 1;
    ++i;
  }
  //f_0
  f0 = vector<const Vertex *> (instance.afss.size() + 1);
  f0[0] = &instance.depot;
  afssF0Indexes[instance.depot.id] = 0;
  int f = 0;
  for (const Vertex& afs : instance.afss) {
    f0[f + 1] = &afs;
    afssF0Indexes[afs.id] = f + 1;
    ++f;
  }
} 

KK_model::~KK_model() {
}

double KK_model::time (int i, int f, int j) const {
  return c0[i]->serviceTime + instance.time(c0[i]->id, f0[f]->id) + (f == 0 ? instance.afss.front().serviceTime : f0[f]->serviceTime) + instance.time(f0[f]->id, c0[j]->id);
}

double KK_model::time(int i, int j) const {
  return c0[i]->serviceTime + instance.time(c0[i]->id, c0[j]->id);
}

double KK_model::customersFuel(int i, int j) const {
  return instance.fuel(c0[i]->id, c0[j]->id);
}

double KK_model::afsToCustomerFuel(int f, int i) const {
  return instance.fuel(f0[f]->id, c0[i]->id);
}

double KK_model::customerToAfsFuel(int i, int f) const {
  return instance.fuel(c0[i]->id, f0[f]->id);
}

double KK_model::M1(int i, int f, int j) {
  return instance.timeLimit + time(i, j) + time(i, f, j) - time(i, 0) - time(0, j);
}

double KK_model::M2(int i, int j) {
  double closestIAFS = afsToCustomerFuel(0, i),
         closestJAFS = afsToCustomerFuel(0, j);
  for (int f = 1; f < f0.size(); ++f) {
    closestIAFS = min(closestIAFS, afsToCustomerFuel(f, i));
    closestJAFS = min(closestJAFS, afsToCustomerFuel(f, j));
  }
  return instance.vehicleFuelCapacity + customersFuel(i, j) - closestIAFS - closestJAFS;
}

pair<Gvrp_solution, Mip_solution_info> KK_model::run(){
  //setup
  stringstream output_exception;
  Mip_solution_info mipSolInfo;
  try {
 //   cout<<"Creating variables"<<endl;
    createVariables();
 //   cout<<"Creating objective function"<<endl;
    createObjectiveFunction();
 //   cout<<"Creating model"<<endl;
    createModel();
 //   cout<<"Setting parameter"<<endl;
    setCustomParameters();
 //   cout<<"Solving model"<<endl;
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
//    cplex.exportModel("cplexcpp.lp");
//    env.out() << "Solution value = " << cplex.getObjValue() << endl;
    elapsed = (finish.tv_sec - start.tv_sec) + (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
    mipSolInfo = Mip_solution_info(cplex.getMIPRelativeGap(), cplex.getStatus(), elapsed, cplex.getObjValue());
    if (RELAXED) {
      endVars ();
      throw mipSolInfo;
    }
 //   cout<<"Getting x values"<<endl;
    fillVals();
 //   cout<<"Creating GVRP solution"<<endl;
    createGvrp_solution();
    endVals();
//    cout<<"Ending vars"<<endl;
    endVars();
    return make_pair(*solution, mipSolInfo);
  } catch (IloException& e) {
    output_exception<<"Concert exception caught: " << e<<endl;
    throw output_exception.str();
  } catch (string s) {
    throw s;
  }
}

void KK_model::createVariables(){
  y = Matrix3DVar (env, c0.size());
  x = Matrix2DVar (env, c0.size());
  e = IloNumVarArray (env, instance.customers.size(), 0, instance.vehicleFuelCapacity, IloNumVar::Float);
  t = IloNumVarArray (env, instance.customers.size(), 0, instance.timeLimit, IloNumVar::Float);
  try {
    //setting names
    stringstream nameStream;
    for (int i = 1; i < c0.size(); ++i) {
      //e var
      nameStream<<"e["<<i - 1<<"]=energy of customer "<<c0[i]->id;
      e[i - 1].setName(nameStream.str().c_str());
      nameStream.clear();
      nameStream.str("");
      //t var
      nameStream<<"t["<<i - 1<<"]=time of customer "<<c0[i]->id;
      t[i - 1].setName(nameStream.str().c_str());
      nameStream.clear();
      nameStream.str("");
    }
    for (int i = 0; i < c0.size(); ++i) {
      //x var
      x[i] = IloNumVarArray (env, c0.size(), 0, 1, RELAXED ? IloNumVar::Float : IloNumVar::Int);
      for (int j = 0; j < c0.size(); ++j) {
        nameStream<<"x["<<i<<"]["<<j<<"]=edge("<<c0[i]->id<<","<<c0[j]->id<<")";
        x[i][j].setName(nameStream.str().c_str());
        nameStream.clear();
        nameStream.str("");
      }
      //y var
      y[i] = Matrix2DVar (env, f0.size());
      for (int f = 0; f < f0.size(); ++f) {
        y[i][f] = IloNumVarArray(env, c0.size(), 0, 1, RELAXED ? IloNumVar::Float : IloNumVar::Int);
        for (int j = 0; j < c0.size(); ++j) {
          nameStream<<"y["<<i<<"]["<<f<<"]["<<j<<"]=path("<<c0[i]->id<<","<<f0[f]->id<<","<<c0[j]->id<<")";
          y[i][f][j].setName(nameStream.str().c_str());
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

void KK_model::createObjectiveFunction() {
  //objective function
  try{
    IloExpr fo (env);
    for (int i = 0; i < c0.size(); ++i) 
      for (int j = 0; j < c0.size(); ++j) {
        fo +=  instance.distances[c0[i]->id][c0[j]->id] * x[i][j];
        for (int f = 0; f < f0.size(); ++f)
          fo += (instance.distances[c0[i]->id][f0[f]->id] + instance.distances[f0[f]->id][c0[j]->id]) * y[i][f][j];
      }
    model = IloModel (env);
    model.add(IloMinimize(env, fo));
  } catch (IloException& e) {
    throw e;
  } catch(...){
    throw string("Error in creating the objective function");
  }
}

void KK_model::createModel() {
  try {
    //constraints
    IloExpr expr(env),
            expr1(env);    
    IloConstraint c;
    stringstream constraintName;
    //x_{ii} = 0, \forall v_i \in C_0
    for (int i = 0; i < c0.size(); ++i) 
      model.add(x[i][i] == 0);
    //y_{ifi} = 0, \forall v_i \in C_0, \forall v_f \in F
    for (int i = 0; i < c0.size(); ++i) 
      for (int f = 0; f < f0.size(); ++f)
        model.add(y[i][f][i] == 0);
    //y_{00i} = y_{i00} = 0, \forall v_i \in C_0
    for (int i = 0; i < c0.size(); ++i) {
      model.add(y[0][0][i] == 0);
      model.add(y[i][0][0] == 0);
    }
    //\sum_{v_j \in C_0} (x_{ij} + \sum_{v_f \in F_0} y_{ifj}) = 1, \forall v_i \in C
    for (int i = 1; i < c0.size(); ++i) {
      for (int j = 0; j < c0.size(); ++j) {
        expr += x[i][j];
        for (int f = 0; f < f0.size(); ++f)
          expr += y[i][f][j];
      }
      c = IloConstraint (expr == 1);
      constraintName<<"customer "<<c0[i]->id<<" must be visited exactly once";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{v_j \in C_0} ((x_{ij} - x_{ji}) + \sum_{v_f \in F_0} (y_{ifj} - y_{jfi})) = 0, \forall v_i \in C_0 
    for (int i = 0; i < c0.size(); ++i) {
      for (int j = 0; j < c0.size(); ++j) {
        expr += x[i][j] - x[j][i];
        for (int f = 0; f < f0.size(); ++f)
          expr += y[i][f][j] - y[j][f][i];
      }
      c = IloConstraint (expr == 0);
      constraintName<<c0[i]->id<<" flow conservation ";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{v_j \in C_0} (x_{0j} + \sum_{v_f \in F_0} y_{0fj}) \leqslant m 
    for (int j = 0; j < c0.size(); ++j) {
      expr += x[0][j];
      for (int f = 0; f < f0.size(); ++f)
        expr += y[0][f][j];
    }
    c = IloConstraint (expr <= instance.maxRoutes);
    constraintName<<instance.maxRoutes<<" routes must be used";
    c.setName(constraintName.str().c_str());
    model.add(c);
    expr.end();
    expr = IloExpr(env);
    constraintName.clear();
    constraintName.str("");
    // \tau_i - \tau_j + (M1_{ifj} - t_{ifj}) * x_{ij}
    //                + (M1_{ifj} - t_{ifj} - t_{ij} - t_{ji}) * x_{ji}
    //                + (M1_{ifj} - t_{ij}) * y_{ifj}
    //                + (M1_{ifj} - t_{ij} - t_{ifj} - t_{jfi}) * y_{jfi}
    //                \leqslant M1_{ifj} - t_{ij} - t_{ifj} 
    for (int i = 1; i < c0.size(); ++i)
      for (int j = 1; j < c0.size(); ++j) 
        for (int f = 0; f < f0.size(); ++f) {
          expr = t[i - 1] - t[j - 1] + (M1(i, f, j) - time(i, f, j)) * x[i][j] 
                              + (M1(i, f, j) - time(i, f, j) - time(i, j) - time(j, i)) * x[j][i]
                             + (M1(i, f, j) - time(i, j)) * y[i][f][j] 
                              + (M1(i, f, j) - time(i, j) - time(i, f, j) - time(j, f, i)) * y[j][f][i]
                              - (M1(i, f, j) - time(i, j) - time(i, f, j));
          c = IloConstraint (expr <= 0);
          constraintName<<"time path ("<<c0[i]->id<<", "<<f0[f]->id<<", "<<c0[j]->id<<")";
          c.setName(constraintName.str().c_str());
          model.add(c);
          expr.end();
          expr = IloExpr(env);
          constraintName.clear();
          constraintName.str("");
        }
    // \tau_j \geqslant t_{0j}) * x_{0j} + \sum_{v_f \in F_0} y_{0fj} t_{0fj} \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      expr = time(0, j) * x[0][j];
      for (int f = 0; f < f0.size(); ++f) 
        expr += time(0, f, j) * y[0][f][j];
      c = IloConstraint (t[j - 1] >= expr);
      constraintName<<"customer "<<c0[j]->id<<" time lb";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    // \tau_j \leqslant T_{max} - (T_{max}^- t_{0j}) * x_{0j} - \sum_{v_f \in F_0} y_{0fj} * (T_{max} - t_{0fj}) \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      expr = instance.timeLimit - (instance.timeLimit - time(0, j)) * x[0][j];
      for (int f = 1; f < f0.size(); ++f) 
        expr -= (instance.timeLimit - time(0, f, j)) * y[0][f][j];
      c = IloConstraint (t[j - 1] <= expr);
      constraintName<<"customer "<<c0[j]->id<<" time ub";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    // \tau_j \leqslant T_{max} - t_{j0} * x_{j0} - \sum_{v_f \in F_0} - t_{jf0} * y_{jf0} \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      expr = instance.timeLimit - time(j, 0) * x[j][0];
      for (int f = 1; f < f0.size(); ++f) 
        expr -= time(j, f, 0) * y[j][f][0];
      c = IloConstraint (t[j - 1] <= expr);
      constraintName<<"customer "<<c0[j]->id<<" time ub 2";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    // e_j - e_i + M2_{ij} * x_{ij} + (M2_{ij} - e_{ij} - e_{ji}) * x_{ji} \leqslant M2_{ij} - e_{ij} \forall v_i, v_j \in C
    for (int i = 1; i < c0.size(); ++i) 
      for (int j = 1; j < c0.size(); ++j) {
        expr = e[j - 1] - e[i - 1] + M2(i, j) * x[i][j] + (M2(i, j) - customersFuel(i, j) - customersFuel(j, i)) * x[j][i];
        c = IloConstraint (expr <= M2(i, j) - customersFuel(i, j));
        constraintName<<"edge ("<<c0[i]->id<<", "<<c0[j]->id<<") energy";
        c.setName(constraintName.str().c_str());
        model.add(c);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    // e_j \leqslant \beta - e_{0j} * x_{0j} - \sum_{v_i \in C_0} \sum_{v_f \in F_0} e_{fj} * y_{ifj} \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      expr = instance.vehicleFuelCapacity - afsToCustomerFuel(0, j) * x[0][j];
      for (int i = 0; i < c0.size(); ++i) 
        for (int f = 0; f < f0.size(); ++f) 
          expr -= afsToCustomerFuel(f, j) * y[i][f][j];
      c = IloConstraint (e[j - 1] <= expr);
      constraintName<<"customer "<<c0[j]->id<<" energy ub";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    // e_j \geqslant e_{j0} * x_{j0} + \sum_{v_i \in C_0} \sum_{v_f \in F_0} e_{jf} * y_{jfi} \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      expr = customerToAfsFuel(j, 0) * x[j][0];
      for (int i = 0; i < c0.size(); ++i) 
        for (int f = 0; f < f0.size(); ++f) 
          expr += customerToAfsFuel(j, f) * y[j][f][i];
      c = IloConstraint (e[j - 1] >= expr);
      constraintName<<"customer "<<c0[j]->id<<" energy ub2";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    // y_{0fi} = y_{if0} = 0 \forall v_i \in C_0, \forall v_f \in F_0 :  e_{0f} > \beta
    for (int f = 0; f < f0.size(); ++f) 
      if (customerToAfsFuel(0, f) > instance.vehicleFuelCapacity)
        for (int i = 0; i < c0.size(); ++i) {
          c = IloConstraint (y[0][f][i] == 0);
          constraintName<<"preprocessing y[0]["<<f<<"]["<<i<<"]";
          c.setName(constraintName.str().c_str());
          model.add(c);
          constraintName.clear();
          constraintName.str("");
          c = IloConstraint (y[i][f][0] == 0);
          constraintName<<"preprocessing y["<<i<<"]["<<f<<"][0]";
          c.setName(constraintName.str().c_str());
          model.add(c);
          constraintName.clear();
          constraintName.str("");
        }



    /*
    vector<vector<int>> routes_ = {
      {0, 5, 9, 21, 9, 34, 9, 38, 0},
      {0, 6, 7, 26, 7, 43, 7, 0, },
      {0, 22, 8, 22, 31, 22, 0},
      {0, 11, 2, 29, 16, 2, 32, 0, },
      {0, 12, 37, 15, 33, 39, 10, 30, 9, 0, },
      {0, 17, 44, 42, 19, 41, 4, 18, 0, },
      {0, 35, 20, 3, 22, 28, 22, 1, 27, 0, },
      {0, 14, 25, 14, 24, 23, 7, 0}
    };
    list<list<Vertex>> routes;
    for (const vector<int>& route_ : routes_) {
      list<Vertex> route;
      for (int node : route_) 
        if (customersC0Indexes.count(node))
          route.push_back(Vertex(*c0[customersC0Indexes[node]]));
        else if (afssF0Indexes.count(node))
          route.push_back(Vertex(*f0[afssF0Indexes[node]]));
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
          currFuel -= customersFuel(i, j);
          currTime += time(i, j);
        } else {
          //is an afs 
          int f = afssF0Indexes[curr->id];
          ++curr;
          int j = customersC0Indexes[curr->id];
          model.add(y[i][f][j] == 1);
          currFuel = instance.vehicleFuelCapacity - afsToCustomerFuel(f, j);
          currTime += time(i, f, j);
        }
      }
    }
    */








    //init
    cplex = IloCplex(model);
    //extra steps
    extraStepsAfterModelCreation();
  } catch (IloException& e) {
    throw e;
  } catch (string s) {
    throw s;
  }
}

void KK_model::extraStepsAfterModelCreation() {
  //
}

void KK_model::setCustomParameters(){
  try{
    setParameters();
    cplex.setParam(IloCplex::Param::Threads, 1);
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in setting parameters");
  }
}

void KK_model::fillVals(){
  //getresult
  try{
    x_vals = Matrix2DVal (env, c0.size());
    y_vals = Matrix3DVal (env, c0.size());
    for (int i = 0; i < c0.size(); ++i){
      x_vals[i] = IloNumArray (env, c0.size(), 0, 1, IloNumVar::Int);
      y_vals[i] = Matrix2DVal (env, f0.size());
      cplex.getValues(x_vals[i], x[i]);
      for (int f = 0; f < f0.size(); ++f){
        y_vals[i][f] = IloNumArray(env, c0.size(), 0, 1, IloNumVar::Int);
        cplex.getValues(y_vals[i][f], y[i][f]);
      }
    }
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting solution");
  }
  /*
  cout<<"Time: "<<endl;
  for (int i = 1; i < c0.size(); ++i)
    cout<<"\t"<<i<<": "<<t[i - 1]<<endl;
  cout<<" ";
  for (int i = 0; i < c0.size(); ++i){
    cout<<" ";
    if (i <=9)
      cout<<" ";
    cout<<i;
  }
  cout<<endl;
  for (int i = 0; i < c0.size(); ++i){
    cout<<i<<" ";
    if (i <= 9)
      cout<<" ";
    for (int j = 0; j < c0.size(); ++j) {
      cout<<abs(x_vals[i][j])<<"  ";
    }
    cout<<endl;
  }
  for (int f = 0; f < f0.size(); ++f){
    cout<<"AFS: "<<f<<endl;
    cout<<" ";
    for (int i = 0; i < c0.size(); ++i){
      cout<<" ";
      if (i <=9)
        cout<<" ";
      cout<<i;
    }
    cout<<endl;
    for (int i = 0; i < c0.size(); ++i){
      cout<<i<<" ";
      if (i <= 9)
        cout<<" ";
      for (int j = 0; j < c0.size(); ++j)
        cout<<abs(y_vals[i][f][j])<<"  ";
      cout<<endl;
    }
  }
  for (int i = 0; i < c0.size(); ++i)
    cout<<i<<": "<<c0[i]->id<<endl;
    */
}

void KK_model::createGvrp_solution(){
  try{
    list<list<Vertex>> routes;
    list<Vertex> route;
    int curr;    
    //checking the depot neighboring
    while (true) {
      bool next = false;
      for (int i = 1; i < c0.size() && !next; ++i) {
        if (x_vals[0][i] > INTEGRALITY_TOL) {
          next = true;
          route.push_back(Vertex(*c0[0]));
          x_vals[0][i] = 0;
        } else
          for (int f = 0; f < f0.size(); ++f)
            if (y_vals[0][f][i] > INTEGRALITY_TOL) {
              next = true;
              route.push_back(Vertex(*c0[0]));
              route.push_back(Vertex(*f0[f]));
              y_vals[0][f][i] = 0;
              break;
            }
        if (next) {
          route.push_back(Vertex(*c0[i]));
          curr = i;
        }
      }
      if (!next)
        break;
      //dfs
      while (curr != 0) {
        for (int i = 0; i < c0.size(); ++i) {
          if (x_vals[curr][i] > INTEGRALITY_TOL) {
            route.push_back(Vertex(*c0[i]));
            x_vals[curr][i] = 0;
            curr = i;
            break;
          } else
            for (int f = 0; f < f0.size(); ++f)
              if (y_vals[curr][f][i] > INTEGRALITY_TOL) {
                next = true;
                route.push_back(Vertex(*f0[f]));
                route.push_back(Vertex(*c0[i]));
                y_vals[curr][f][i] = 0;
                curr = i;
                i = c0.size() - 1;
                break;
              }
        }
      }
      routes.push_back(route);
      route = list<Vertex> ();
    }
    solution = new Gvrp_solution(routes, instance);
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting routes");
  }
}

void KK_model::endVals () {
  //end vals
  for (int i = 0; i < c0.size(); ++i) {
    for (int f = 0; f < f0.size(); ++f)
      y_vals[i][f].end();
    y_vals[i].end();
    x_vals[i].end();
  }
  y_vals.end();
  x_vals.end();
}

void KK_model::endVars(){
  for (int i = 0; i < c0.size(); ++i) {
    x[i].end();
    for (int f = 0; f < f0.size(); ++f) 
      y[i][f].end();
    y[i].end();
  }
  x.end();
  y.end();
  e.end(); 
  t.end(); 
}
