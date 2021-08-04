#include "models/vertex.hpp"
#include "models/gvrp_models/cplex/kk_model/kk_model.hpp"
#include "models/gvrp_models/cplex/kk_model/mip_start.hpp"

#include <iostream>
#include <string>
#include <ilcplex/ilocplex.h>

ILOSTLBEGIN
using namespace std;
using namespace models;
using namespace models::gvrp_models::cplex::kk_model;

Mip_start::Mip_start (const Gvrp_instance& instance, unsigned int time_limit, const Gvrp_solution& gvrp_solution_) : KK_model (instance, time_limit), gvrp_solution (gvrp_solution_) {
}

void Mip_start::extraStepsAfterModelCreation () {
  try {
    KK_model::extraStepsAfterModelCreation();
    //setup
    IloNumArray t_vals = IloNumArray (env, c0.size() - 1, 0, instance.timeLimit, IloNumVar::Float),
                e_vals = IloNumArray (env, c0.size() - 1, 0, instance.vehicleFuelCapacity, IloNumVar::Float);
    x_vals = Matrix2DVal (env, c0.size());
    y_vals = Matrix3DVal (env, c0.size());
    //create vals
    for (int i = 0; i < c0.size(); ++i) {
      //x
      x_vals[i] = IloNumArray (env, c0.size(), 0, 1, IloNumVar::Int);
      for (int j = 0; j < c0.size(); ++j) 
        //x
        x_vals[i][j] = 0.0;
      //y var
      y_vals[i] = Matrix2DVal (env, f0.size());
      for (int f = 0; f < f0.size(); ++f) {
        y_vals[i][f] = IloNumArray(env, c0.size(), 0, 1, IloNumVar::Int);
        for (int j = 0; j < c0.size(); ++j) 
          y_vals[i][f][j] = 0.0;
      }
    }
    //get values
    double currFuel, 
           currTime;
    for (const list<Vertex>& route : gvrp_solution.routes) {
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
          x_vals[i][j] = 1;
          currFuel -= customersFuel(i, j);
          currTime += time(i, j);
          if (j > 0) {
            e_vals[j - 1] = currFuel;
            t_vals[j - 1] = currTime;
          }
        } else {
          //is an afs 
          int f = afssF0Indexes[curr->id];
          ++curr;
          int j = customersC0Indexes[curr->id];
          y_vals[i][f][j] = 1;
          currTime += time(i, f, j);
          currFuel = instance.vehicleFuelCapacity - afsToCustomerFuel(f, j);
          if (j > 0) {
            e_vals[j - 1] = currFuel;
            t_vals[j - 1] = currTime;
          }
        }
      }
    }
    //mip start
    IloCplex::MIPStartEffort effort = IloCplex::MIPStartCheckFeas;
    IloNumVarArray startVar(env);
    IloNumArray startVal(env);
    for (int i = 0; i < c0.size(); ++i) {
      //e and t
      if (i > 0) {
        startVar.add(e[i - 1]);
        startVal.add(e_vals[i - 1]);
        startVar.add(t[i - 1]);
        startVal.add(t_vals[i - 1]);
      }
      for (int j = 0; j < c0.size(); ++j) {
        //x
        startVar.add(x[i][j]);
        startVal.add(x_vals[i][j]);
      }
      //y 
      for (int f = 0; f < f0.size(); ++f) 
        for (int j = 0; j < c0.size(); ++j) {
          startVar.add(y[i][f][j]);
          startVal.add(y_vals[i][f][j]);
        }
    }
    cplex.addMIPStart (startVar, startVal, effort);
    //clean vals
    for (int i = 0; i < c0.size(); ++i) {
      x_vals[i].end();
      for (int f = 0; f < f0.size(); ++f) 
        y_vals[i][f].end();
      y_vals[i].end();
    }
    x_vals.end();
    y_vals.end();
    e_vals.end();
    t_vals.end();
    startVar.end();
    startVal.end();
  } catch (IloException& e) {
    throw e;
  } catch (string& s) {
    throw s;
  } catch (...) {
    throw string("Error in setting MIP start solution");
  }
}

