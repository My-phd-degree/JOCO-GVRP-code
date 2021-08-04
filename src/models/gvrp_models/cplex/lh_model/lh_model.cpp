#include "models/vertex.hpp"
#include "models/distances_enum.hpp"
#include "models/cplex/mip_solution_info.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/cplex/gvrp_model.hpp"
#include "models/gvrp_models/cplex/lh_model/lh_model.hpp"
#include "models/gvrp_models/cplex/lh_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/lh_model/invalid_edge_preprocessing.hpp"
#include "models/gvrp_models/cplex/lh_model/invalid_edge_preprocessing_C4_C5.hpp"
#include "models/gvrp_models/cplex/lh_model/invalid_edge_preprocessing_C6.hpp"
#include "models/gvrp_models/cplex/lh_model/invalid_edge_preprocessing_C7.hpp"

#include <sstream>
#include <list>
#include <time.h> 
#include <string> 

using namespace models;
using namespace models::cplex;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex;
using namespace models::gvrp_models::cplex::lh_model;

using namespace std;

LH_model::LH_model(const Gvrp_instance& instance, unsigned int time_limit) : Gvrp_model(instance, time_limit), nPreprocessings0(0), nPreprocessings1(0), nPreprocessings2(0), nPreprocessings3(0), RELAXED(false) {
  if (instance.distances_enum != METRIC)
    throw string("Error: The compact model requires a G-VRP instance with metric distances");
  //c_0
  c0 = vector<const Vertex *> (instance.customers.size() + 1);
  c0[0] = &instance.depot;
  customersC0Indexes[instance.depot.id] = 0;
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
  //preprocessing
  preprocessings.push_back(new Invalid_edge_preprocessing(*this));
  preprocessings.push_back(new Invalid_edge_preprocessing_C4_C5(*this));
  preprocessings.push_back(new Invalid_edge_preprocessing_C6(*this));
  preprocessings.push_back(new Invalid_edge_preprocessing_C7(*this));
} 

LH_model::~LH_model() {
  for (Preprocessing * preprocessing : preprocessings)
    delete preprocessing;  
}

double LH_model::time (int i, int f, int j) {
  return c0[i]->serviceTime + instance.time(c0[i]->id, f0[f]->id) + (f == 0 ? instance.afss.front().serviceTime : f0[f]->serviceTime) + instance.time(f0[f]->id, c0[j]->id);
}

double LH_model::time(int i, int j) {
  return c0[i]->serviceTime + instance.time(c0[i]->id, c0[j]->id);
}

double LH_model::customersFuel(int i, int j) {
  return instance.fuel(c0[i]->id, c0[j]->id);
}

double LH_model::afsToCustomerFuel(int f, int i) {
  return instance.fuel(f0[f]->id, c0[i]->id);
}

double LH_model::customerToAfsFuel(int i, int f) {
  return instance.fuel(c0[i]->id, f0[f]->id);
}

pair<Gvrp_solution, Mip_solution_info> LH_model::run(){
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
    fillVals();
//    cout<<"Creating GVRP solution"<<endl;
    createGvrp_solution();
    endVals ();
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

void LH_model::createVariables(){
  y = Matrix3DVar (env, c0.size());
  x = Matrix2DVar (env, c0.size());
  a = Matrix2DVar (env, c0.size());
  u = Matrix2DVar (env, c0.size());
  v = Matrix2DVar (env, c0.size() - 1);
  try {
    //setting names
    stringstream nameStream;
    for (int i = 0; i < c0.size(); ++i) {
      //x, u, v, and a vars
      x[i] = IloNumVarArray (env, c0.size(), 0, 1, RELAXED ? IloNumVar::Float : IloNumVar::Int);
      a[i] = IloNumVarArray (env, c0.size() - 1, 0, instance.vehicleFuelCapacity, IloNumVar::Float);
      u[i] = IloNumVarArray (env, c0.size(), 0, instance.timeLimit, IloNumVar::Float);
      //a
      if (i > 0) {
        v[i - 1] = IloNumVarArray (env, f0.size(), 0, instance.vehicleFuelCapacity, IloNumVar::Float);
        for (int f = 0; f < f0.size(); ++f) {
          nameStream<<"v["<<i - 1<<"]["<<f<<"]=edge("<<c0[i]->id<<","<<f0[f]->id<<")";
          v[i - 1][f].setName(nameStream.str().c_str());
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

void LH_model::createObjectiveFunction() {
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

void LH_model::createModel() {
  try {
    //preprocessing conditions
    for (Preprocessing* preprocessing : preprocessings)
      preprocessing->add();
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
        for (int f = 1; f < f0.size(); ++f)
          expr += y[i][f][j];
      }
      c = IloConstraint (expr == 1);
      constraintName<<"# of exiting edges in customer "<<c0[i]->id<<" must exactly one";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{v_j \in C_0} (x_{ji} + \sum_{v_f \in F_0} y_{jfi}) = 1, \forall v_i \in C
    for (int i = 1; i < c0.size(); ++i) {
      for (int j = 0; j < c0.size(); ++j) {
        expr += x[j][i];
        for (int f = 1; f < f0.size(); ++f)
          expr += y[j][f][i];
      }
      c = IloConstraint (expr == 1);
      constraintName<<"# of entering edges in customer "<<c0[i]->id<<" must exactly one";
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
    constraintName<<"at most "<<instance.maxRoutes<<" routes must be used";
    c.setName(constraintName.str().c_str());
    model.add(c);
    expr.end();
    expr = IloExpr(env);
    constraintName.clear();
    constraintName.str("");
    //time constraints
    //\sum_{v_i \in C \cup \{v_0\}} u_{ji} = \sum_{v_i \in C \cup \{v_0\}} u_{ij} + \sum_{v_i \in C \cup \{v_0\}} t_{ij} x_{ij} + \sum_{v_i \in C \cup \{v_0\}} \sum_{v_f \in F_0} t_{irj} x_{irj}, \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      for (int i = 0; i < c0.size(); ++i) {
        expr += u[j][i] - u[i][j] - time(i, j) * x[i][j];
        for (int f = 1; f < f0.size(); ++f)
          expr -= time(i, f, j) * y[i][f][j];
      }
      c = IloConstraint (expr == 0);
      constraintName<<"time in customer "<<c0[j]->id;
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //t_{0j}(x_{j0} + \sum_{v_f \in F} t_{jf0}) \leqslant u_{j0}, \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      expr = x[j][0];
      for (int f = 1; f < f0.size(); ++f)
        expr += y[j][f][0];
      c = IloConstraint (time(0, j) * expr <= u[j][0]);
      constraintName<<"lb time in customer "<<c0[j]->id<<" to depot";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //u_{j0} \leqslant (T - t_{j0}) x_{j0} + \sum_{v_f \in F} (T - t_{jf0}) y_{jf0}, \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      expr = (instance.timeLimit - time(j, 0)) * x[j][0];
      for (int f = 1; f < f0.size(); ++f)
        expr += (instance.timeLimit - time(j, f, 0)) * y[j][f][0];
      c = IloConstraint (expr >= u[j][0]);
      constraintName<<"ub time in customer "<<c0[j]->id<<" to depot";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //u_{0j} = 0, \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      c = IloConstraint (u[0][j] == 0);
      constraintName<<"time in depot to "<<c0[j]->id<<" must be 0";
      c.setName(constraintName.str().c_str());
      model.add(c);
      constraintName.clear();
      constraintName.str("");
    }
    //u_{ij} \geqslant max(t_{0j} - t_{ij}, t_{0i}) x_{ij} + \sum_{v_f \in F} max(t_{0j} - t_{ifj}, t_{0i}) y_{ifj}, \forall v_i, v_j \in C
    for (int i = 1; i < c0.size(); ++i) 
      for (int j = 1; j < c0.size(); ++j) {
        expr = max(time(0, j) - time(i, j), time(0, i)) * x[i][j];
        for (int f = 1; f < f0.size(); ++f)
          expr += max(time(0, j) - time(i, f, j), time(0, i)) * y[i][f][j];
        c = IloConstraint (u[i][j] >= expr);
        constraintName<<"lb time in "<<c0[i]->id<<" to "<<c0[j]->id;
        c.setName(constraintName.str().c_str());
        model.add(c);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    //u_{ij} \leqslant max(T - t_{j0} - t_{ij}, T - t_{i0}) x_{ij} + \sum_{v_f \in F} max(T - t_{j0} - t_{ifj}, T - t_{i0}) y_{ifj}, \forall v_i, v_j \in C
    for (int i = 1; i < c0.size(); ++i) 
      for (int j = 1; j < c0.size(); ++j) {
        expr = min(instance.timeLimit - time(j, 0) - time(i, j), instance.timeLimit - time(i, 0)) * x[i][j];
        for (int f = 1; f < f0.size(); ++f)
          expr += min(instance.timeLimit - time(j, 0) - time(i, f, j), instance.timeLimit - time(i, 0)) * y[i][f][j];
        c = IloConstraint (u[i][j] <= expr);
        constraintName<<"lb time in "<<c0[i]->id<<" to "<<c0[j]->id;
        c.setName(constraintName.str().c_str());
        model.add(c);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    //energy constraints
    //\sum_{v_i \in C} a_{i, j} - a_{j, i} = \sum_{v_f \in F_0} v_{jr} + \sum_{v_i \in C} e_{ji} x_{ji} - \sum_{v_i \in C_0} \sum_{v_f \in F} (\beta - e_{fj}) y_{ifj} - (\beta - e_{0j}) x_{0j}, \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      for (int f = 0; f < f0.size(); ++f)
        expr -= v[j - 1][f];
      for (int i = 0; i < c0.size(); ++i) {
        if (i > 0) 
          expr += a[i - 1][j - 1] - a[j - 1][i - 1] - customersFuel(j, i) * x[j][i];
        for (int f = 1; f < f0.size(); ++f)
          expr += (instance.vehicleFuelCapacity - afsToCustomerFuel(f, j)) * y[i][f][j];
      }
      expr += (instance.vehicleFuelCapacity - customersFuel(0, j)) * x[0][j];
      c = IloConstraint (expr == 0);
      constraintName<<"customer "<<c0[j]->id<<" energy update";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //a_{ij} >= x_{ij} * max (min_{v_f \in F_0} e_{jf}, min_{v_f \in F_0} e_{if} - e_{ij}), \forall v_i, v_j \in C
    for (int j = 1; j < c0.size(); ++j) 
      for (int i = 1; i < c0.size(); ++i) {
        //min_{v_f \in F_0} e_{jf} and min_{v_f \in F_0} e_{if}
        double closestAfsToJ = customerToAfsFuel(j, 0),
               closestAfsToI = customerToAfsFuel(i, 0);
        for (int f = 1; f < f0.size(); ++f) {
          closestAfsToJ = min(closestAfsToJ, customerToAfsFuel(j, f));
          closestAfsToI = min(closestAfsToI, customerToAfsFuel(i, f));
        }
        c = IloConstraint (a[i - 1][j - 1] >= x[i][j] * max(closestAfsToJ, closestAfsToI - customersFuel(i, j)));
        constraintName<<"a["<<i - 1<<"]["<<j - 1<<"] lb";
        c.setName(constraintName.str().c_str());
        model.add(c);
        constraintName.clear();
        constraintName.str("");
      }
    //a_{ij} <= x_{ij} * min (\beta - min_{v_f \in F_0} e_{fj}, \beta - min_{v_f \in F_0} e_{fi} - e_{ij}), \forall v_i, v_j \in C
    for (int j = 1; j < c0.size(); ++j) 
      for (int i = 1; i < c0.size(); ++i) {
        //min_{v_f \in F_0} e_{fj} and min_{v_f \in F_0} e_{fi}
        double closestAfsToJ = afsToCustomerFuel(0, j),
               closestAfsToI = afsToCustomerFuel(0, i);
        for (int f = 1; f < f0.size(); ++f) {
          closestAfsToJ = min(closestAfsToJ, afsToCustomerFuel(f, j));
          closestAfsToI = min(closestAfsToI, afsToCustomerFuel(f, i));
        }
        c = IloConstraint (a[i - 1][j - 1] <= x[i][j] * min(instance.vehicleFuelCapacity - closestAfsToI - customersFuel(i, j), instance.vehicleFuelCapacity - closestAfsToJ));
        constraintName<<"a["<<i - 1<<"]["<<j - 1<<"] ub";
        c.setName(constraintName.str().c_str());
        model.add(c);
        constraintName.clear();
        constraintName.str("");
      }
    //v_{j0} \geqslant x_{j0} * e_{j0}, \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      c = IloConstraint (v[j - 1][0] >= x[j][0] * customersFuel(j, 0));
      constraintName<<"v["<<j - 1<<"][0] lb";
      c.setName(constraintName.str().c_str());
      model.add(c);
      constraintName.clear();
      constraintName.str("");
    }
    //v_{j0} \leqslant x_{j0} * (\beta - min_{v_f \in F_0} e_{fj}), \forall v_j \in C
    for (int j = 1; j < c0.size(); ++j) {
      //min_{v_f \in F_0} e_{fj}
      double closestAfsToJ = afsToCustomerFuel(0, j);
      for (int f = 1; f < f0.size(); ++f) 
        closestAfsToJ = min(closestAfsToJ, afsToCustomerFuel(f, j));
      c = IloConstraint (v[j - 1][0] <= x[j][0] * (instance.vehicleFuelCapacity - closestAfsToJ));
      constraintName<<"v["<<j - 1<<"][0] ub";
      c.setName(constraintName.str().c_str());
      model.add(c);
      constraintName.clear();
      constraintName.str("");
    }
    //v_{jr} \geqslant \sum_{v_i \in C_0} z_{jfi} * e_{jf}, \forall v_j \in C, \forall v_f \in F
    for (int j = 1; j < c0.size(); ++j) {
      for (int f = 1; f < f0.size(); ++f) {
        for (int i = 0; i < c0.size(); ++i) 
          expr += y[j][f][i];
        c = IloConstraint (v[j - 1][f] >= expr * customerToAfsFuel(j, f));
        constraintName<<"v["<<j - 1<<"]["<<f<<"] lb";
        c.setName(constraintName.str().c_str());
        model.add(c);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    }
    //v_{jr} \leqslant (\beta - min_{v_f \in F_0} e_{fj}) * \sum_{v_i \in C_0} z_{jfi}, \forall v_j \in C, \forall v_f \in F
    for (int j = 1; j < c0.size(); ++j) {
      for (int f = 1; f < f0.size(); ++f) {
        for (int i = 0; i < c0.size(); ++i) 
          expr += y[j][f][i];
        //min_{v_f \in F_0} e_{jf}
        double closestAfsToJ = customerToAfsFuel(j, 0);
        for (int f = 1; f < f0.size(); ++f) 
          closestAfsToJ = min(closestAfsToJ, customerToAfsFuel(j, f));
        c = IloConstraint (v[j - 1][f] <= expr * (instance.vehicleFuelCapacity - closestAfsToJ));
        constraintName<<"v["<<j - 1<<"]["<<f<<"] ub";
        c.setName(constraintName.str().c_str());
        model.add(c);
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
        for (int f = 1; f < f0.size(); ++f) 
          expr += y[j][f][i] + y[i][f][j];
        c = IloConstraint (expr <= 1);
        constraintName<<"no 2 subcyle between customers "<<i<<", and "<<j;
        c.setName(constraintName.str().c_str());
        model.add(c);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    }


    /*
    vector<vector<int>> routes_ = {
      {1, 11, 31, 30, 1},
      {1, 13, 15, 24, 16, 14, 22, 20, 1},
      {1, 5, 12, 28, 18, 17, 1},
      {1, 23, 27, 25, 1},
      {1, 19, 2, 29, 26, 21, 1},
    };
    for (vector<int>& route : routes_) {
      for (size_t i = 0; i < route.size(); ++i) {
        --route[i];
      }
    }
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
          model.add(u[i][j] == currTime);
          if (i > 0 && j > 0) 
            model.add(a[i - 1][j - 1] == currFuel - customersFuel(i, j));
          else if (i > 0) 
            model.add(v[i - 1][0] == currFuel);
          currFuel -= customersFuel(i, j);
          currTime += time(i, j);
        } else {
          //is an afs 
          int f = afssF0Indexes[curr->id];
          ++curr;
          int j = customersC0Indexes[curr->id];
          model.add(y[i][f][j] == 1);
          model.add(u[i][j] == currTime);
          if (i > 0) 
            model.add(v[i - 1][f] == currFuel);
          currTime += time(i, f, j);

          currFuel = instance.vehicleFuelCapacity - afsToCustomerFuel(f, j);
        }
      }
      for (const Vertex& vertex : route)
        cout<<vertex.id<<" ";
      cout<<"("<<currFuel<<"), ("<<currTime<<")"<<endl;
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

void LH_model::extraStepsAfterModelCreation() {
  //
}

void LH_model::setCustomParameters(){
  try{
    setParameters();
    cplex.setParam(IloCplex::Param::Threads, 1);
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in setting parameters");
  }
}

void LH_model::fillVals(){
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

void LH_model::createGvrp_solution(){
  try{
    list<list<Vertex>> routes;
    list<Vertex> route;
    int curr;    
    bool next = false;
    //checking the depot neighboring
    while (true) {
      next = false;
      for (int i = 1; i < c0.size() && !next; ++i) {
        if (x_vals[0][i] > INTEGRALITY_TOL) {
          next = true;
          route.push_back(Vertex(*c0[0]));
          x_vals[0][i] = 0;
        } 
        if (!next)
          //on y[j][0][i]
          for (int j = 0; j < c0.size(); ++j) 
            if (y_vals[j][0][i] > INTEGRALITY_TOL) {
              next = true;
              route.push_back(Vertex(*c0[0]));
              y_vals[j][0][i] = 0;
              break;
            }
        if (!next)
          //on y[0][f][i]
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
          next = false;
          if (x_vals[curr][i] > INTEGRALITY_TOL) {
            next = true;
            route.push_back(Vertex(*c0[i]));
            x_vals[curr][i] = 0;
            curr = i;
            break;
          } else {
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
        //edge case
        if (!next) {
          route.push_back(Vertex(*c0[0]));
          curr = 0;
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

void LH_model::endVals () {
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

void LH_model::endVars(){
  for (int i = 0; i < c0.size(); ++i) {
    if (i > 0) {
      a[i - 1].end();
      v[i - 1].end();
    }
    u[i].end();
    x[i].end();
    for (int f = 0; f < f0.size(); ++f) 
      y[i][f].end();
    y[i].end();
  }
  x.end();
  y.end();
  a.end();
  u.end();
  v.end();
}
