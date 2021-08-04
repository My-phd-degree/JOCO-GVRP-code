#include "models/vertex.hpp"
#include "models/cplex/mip_depth.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/matheus_model_4.hpp"
#include "models/gvrp_models/cplex/matheus_model_4/greedy_lp_heuristic.hpp"
#include "models/gvrp_models/cplex/gvrp_lp_kk_heuristic.hpp"
#include "models/gvrp_models/local_searchs/fsRemove.hpp"

#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <cmath>
#include <list>
#include <ilcplex/ilocplex.h>
#include <iostream>

ILOSTLBEGIN

using namespace std;
using namespace models;
using namespace models::cplex;
using namespace models::gvrp_models::local_searchs;
using namespace models::gvrp_models::cplex;
using namespace models::gvrp_models::cplex::matheus_model_4;

Greedy_lp_heuristic::Greedy_lp_heuristic (Matheus_model_4& matheus_model_4) : Heuristic_callback (matheus_model_4) {}

IloCplex::CallbackI* Greedy_lp_heuristic::duplicateCallback() const {
  return new(getEnv()) Greedy_lp_heuristic (*this);
}

void Greedy_lp_heuristic::main() {
  try {
    //node callback
    Depth const* const d = (Depth *) getNodeData();
    IloInt depth = d ? d->depth : 0;
    if (depth > 10 && depth%10 == 0) 
      return;
    //setup
    const int sc0 = matheus_model_4.c0.size(),
          s_f = matheus_model_4._f.size();
    IloEnv env = getEnv();
    IloExpr lhs (env);
    list<list<Vertex>> routes;
    list<Vertex> route;
    int curr;    
    unordered_set<int> customers;
    bool valid = true,
         found,
         frac_sol = false;
    double maxFirst,
           remainingFuel,
           remainingTime,
           cost = 0;
    int nextCustomer;
    pair<int, int> nextAFSs;
    matheus_model_4.x_vals = Matrix2DVal (env, sc0);
    matheus_model_4.y_vals = Matrix4DVal (env, sc0);
    //get values
    for (int i = 0; i < sc0; ++i) {
      matheus_model_4.x_vals[i] = IloNumArray (env, sc0, 0, 1, IloNumVar::Float);
      matheus_model_4.y_vals[i] = Matrix3DVal (env, s_f);
      getValues(matheus_model_4.x_vals[i], matheus_model_4.x[i]);
      //check y values
      if (!frac_sol)
        for (int j = 0; j < sc0; ++j)
          if (abs(round(matheus_model_4.x_vals[i][j]) - matheus_model_4.x_vals[i][j]) > matheus_model_4.INTEGRALITY_TOL) 
            frac_sol = true;
      //get y vals
      for (int f = 0; f < s_f; ++f) {
        matheus_model_4.y_vals[i][f] = Matrix2DVal (env, s_f);
        for (int r = 0; r < s_f; ++r) {
          matheus_model_4.y_vals[i][f][r] = IloNumArray (env, sc0, 0, 1, IloNumVar::Float);
          getValues(matheus_model_4.y_vals[i][f][r], matheus_model_4.y[i][f][r]);
          //check y values
          if (!frac_sol)
            for (int j = 0; j < sc0; ++j)
              if (abs(round(matheus_model_4.y_vals[i][f][r][j]) - matheus_model_4.y_vals[i][f][r][j]) > matheus_model_4.INTEGRALITY_TOL) 
                frac_sol = true;
        }
      }
    }
    if (!frac_sol) 
      return;
    //checking the depot neighboring
    while (true) {
      //setup
      maxFirst = -1;
      remainingFuel = matheus_model_4.instance.vehicleFuelCapacity;
      remainingTime = matheus_model_4.instance.timeLimit;
      nextCustomer = 0;
      nextAFSs = {0, 0};
      found = false;
      //get route beginning
      for (int i = 1; i < matheus_model_4.c0.size(); ++i) 
        if (!customers.count(i)) {
          if (matheus_model_4.x_vals[0][i] > maxFirst) {
            maxFirst = matheus_model_4.x_vals[0][i];
            nextCustomer = i;
            found = true;
          } else
            for (int f = 0; f < matheus_model_4._f.size(); ++f)
              for (int r = 0; r < matheus_model_4._f.size(); ++r)
              if (matheus_model_4.y_vals[0][f][r][i] > maxFirst) {
                maxFirst = matheus_model_4.y_vals[0][f][r][i];
                nextAFSs = {f, r};
                nextCustomer = i;
                found = true;
              }
        }
      if (!found)
        break;
      //update dss
      route.push_back(Vertex(*matheus_model_4.c0[0]));
      //get remaining route
      if (maxFirst != matheus_model_4.x_vals[0][nextCustomer]) {
        if (matheus_model_4.customerToAfsFuel(0, nextAFSs.first) > remainingFuel || matheus_model_4.afsToCustomerFuel(nextAFSs.second, nextCustomer) > remainingFuel || remainingTime - matheus_model_4.time(0, nextAFSs.first, nextAFSs.second, nextCustomer) < 0) {
          valid = false;
          break;
        }
        remainingFuel -= matheus_model_4.afsToCustomerFuel(nextAFSs.second, nextCustomer);
        remainingTime -= matheus_model_4.time(0, nextAFSs.first, nextAFSs.second, nextCustomer);
        cost += matheus_model_4.distance(0, nextAFSs.first, nextAFSs.second, nextCustomer);
        list<Vertex> path = matheus_model_4.getAFSsShortestPath(*matheus_model_4._f[nextAFSs.first], *matheus_model_4._f[nextAFSs.second]);
        for (const Vertex& afs : path)
          route.push_back(Vertex(afs));
      } else if (matheus_model_4.customersFuel(0, nextCustomer) > remainingFuel || remainingTime - matheus_model_4.time(0, nextCustomer) < 0) {
        valid = false;
        break;
      } else {
        remainingFuel -= matheus_model_4.afsToCustomerFuel(0, nextCustomer);
        remainingTime -= matheus_model_4.time(0, nextCustomer);
      }
      cost += matheus_model_4.instance.distances[route.back().id][matheus_model_4.c0[nextCustomer]->id];
      route.push_back(Vertex(*matheus_model_4.c0[nextCustomer]));
      customers.insert(nextCustomer);
      curr = nextCustomer;
      //dfs
      while (curr != 0) {
        maxFirst = 0;
        nextCustomer = 0;
        nextAFSs = {0, 0};
        found = false;
        for (int i = 0; i < matheus_model_4.c0.size(); ++i) 
          if (!customers.count(i)) {
            if (matheus_model_4.x_vals[curr][i] > maxFirst) {
              maxFirst = matheus_model_4.x_vals[curr][i];
              nextCustomer = i;
              found = true;
            } else
              for (int f = 0; f < matheus_model_4._f.size(); ++f)
                for (int r = 0; r < matheus_model_4._f.size(); ++r)
                  if (matheus_model_4.y_vals[curr][f][r][i] > 0) {
                    maxFirst = matheus_model_4.y_vals[curr][f][r][i];
                    nextAFSs = {f, r};
                    nextCustomer = i;
                    found = true;
                  }
          }
        //repeated customer
        if (!found) {
          valid = false;
          break;
        }
        //update dss
        if (maxFirst != matheus_model_4.x_vals[curr][nextCustomer]) {
          if (matheus_model_4.customerToAfsFuel(curr, nextAFSs.first) > remainingFuel || matheus_model_4.afsToCustomerFuel(nextAFSs.second, nextCustomer) > matheus_model_4.instance.vehicleFuelCapacity || remainingTime - matheus_model_4.time(curr, nextAFSs.first, nextAFSs.second, nextCustomer) < 0) {
            valid = false;
            break;
          }
          remainingFuel = matheus_model_4.instance.vehicleFuelCapacity - matheus_model_4.afsToCustomerFuel(nextAFSs.second, nextCustomer);
          remainingTime -= matheus_model_4.time(curr, nextAFSs.first, nextAFSs.second, nextCustomer);
          cost += matheus_model_4.distance(curr, nextAFSs.first, nextAFSs.second, nextCustomer);
          list<Vertex> path = matheus_model_4.getAFSsShortestPath(*matheus_model_4._f[nextAFSs.first], *matheus_model_4._f[nextAFSs.second]);
          for (const Vertex& afs : path)
            route.push_back(Vertex(afs));
        } else if (matheus_model_4.customersFuel(curr, nextCustomer) > remainingFuel || remainingTime - matheus_model_4.time(curr, nextCustomer) < 0) {
          valid = false;
          break;
        } else {
          remainingFuel -= matheus_model_4.customersFuel(curr, nextCustomer);
          remainingTime -= matheus_model_4.time(curr, nextCustomer);
        }
        cost += matheus_model_4.instance.distances[route.back().id][matheus_model_4.c0[nextCustomer]->id];
        route.push_back(Vertex(*matheus_model_4.c0[nextCustomer]));
        if (nextCustomer != 0)
          customers.insert(nextCustomer);
        curr = nextCustomer;
      }
      if (!valid)
        break;
      routes.push_back(route);
      route = list<Vertex> ();
    }
    if (valid) {
      cout<<"============================HERERERE======================"<<endl;
      Gvrp_solution gvrp_solution (routes, matheus_model_4.instance);

      FsRemove fsRemove (matheus_model_4.instance, gvrp_solution);
      gvrp_solution = fsRemove.run();
      routes = gvrp_solution.routes;

      cost = gvrp_solution.calculateCost();
    }
    //better solution found
    if (valid && cost - getIncumbentObjValue() < -1e-6) {
      ++matheus_model_4.nGreedyLP;
      cout.precision(17);
      cout<<getIncumbentObjValue()<<" to "<<cost<<endl;
      //set new solution
      //reset all the values
      IloNumArray e_vals = IloNumArray (matheus_model_4.env, sc0 - 1, 0, matheus_model_4.instance.vehicleFuelCapacity, IloNumVar::Float),
                  t_vals = IloNumArray (matheus_model_4.env, sc0 - 1, 0, matheus_model_4.instance.timeLimit, IloNumVar::Float);
      matheus_model_4.x_vals = Matrix2DVal (matheus_model_4.env, sc0);
      matheus_model_4.y_vals = Matrix4DVal (matheus_model_4.env, sc0);
      //create vals
      for (int i = 0; i < sc0; ++i) {
        //t and e
        if (i > 0) {
          t_vals[i - 1] = 0.0;
          e_vals[i - 1] = 0.0;
        }
        //x, and y vars
        matheus_model_4.x_vals[i] = IloNumArray (matheus_model_4.env, sc0, 0, 1, IloNumVar::Int);
        for (int j = 0; j < sc0; ++j) 
          matheus_model_4.x_vals[i][j] = 0.0;
        //y var
        matheus_model_4.y_vals[i] = Matrix3DVal (env, s_f);
        for (int f = 0; f < s_f; ++f) {
          matheus_model_4.y_vals[i][f] = Matrix2DVal (env, s_f);
          for (int r = 0; r < s_f; ++r) {
            matheus_model_4.y_vals[i][f][r] = IloNumArray(env, sc0, 0, 1, IloNumVar::Int);
            for (int j = 0; j < sc0; ++j) 
              matheus_model_4.y_vals[i][f][r][j] = 0.0;
          }
        }
      }
      //get values
      IloNumVarArray vars (matheus_model_4.env);
      IloNumArray vals (matheus_model_4.env);
      double currFuel, 
             currTime;
      for (const list<Vertex>& route : routes) {
        currFuel = matheus_model_4.instance.vehicleFuelCapacity;
        currTime = 0.0;
        list<Vertex>::const_iterator curr = route.begin(), 
          prev = curr;
        for (++curr; curr != route.end(); prev = curr, ++curr) {
          auto currIndex = matheus_model_4.customersC0Indexes.find(curr->id);
          int i = matheus_model_4.customersC0Indexes[prev->id];
          //is a customer
          if (currIndex != matheus_model_4.customersC0Indexes.end() && !(curr->id == matheus_model_4.instance.depot.id && curr != --route.end())) {
            int j = currIndex->second;
            matheus_model_4.x_vals[i][j] = 1;
            currFuel -= matheus_model_4.customersFuel(i, j);
            currTime += matheus_model_4.time(i, j);
            if (j != 0) {
              t_vals[j - 1] = currTime;
              e_vals[j - 1] = currFuel;
            }
          } else {
            //is an afs 
            int f = matheus_model_4.afss_FIndexes[curr->id], 
                r = f;
            for (++curr; matheus_model_4.afss_FIndexes.count(curr->id); r = matheus_model_4.afss_FIndexes[curr->id], ++curr);
            int j = matheus_model_4.customersC0Indexes[curr->id];
            matheus_model_4.y_vals[i][f][r][j] = 1;
            currTime += matheus_model_4.time(i, f, r, j);
            currFuel = matheus_model_4.instance.vehicleFuelCapacity - matheus_model_4.afsToCustomerFuel(r, j);
            if (j != 0) {
              t_vals[j - 1] = currTime;
              e_vals[j - 1] = currFuel;
            }
          }
        }
      }
      //set values
      for (int i = 0; i < sc0; ++i) {
        if (i > 0) {
          //t
          vars.add(matheus_model_4.t[i - 1]);
          vals.add(t_vals[i - 1]);
          //e
          vars.add(matheus_model_4.e[i - 1]);
          vals.add(e_vals[i - 1]);
        }
        for (int j = 0; j < sc0; ++j) {
          //x
          vars.add(matheus_model_4.x[i][j]);
          vals.add(matheus_model_4.x_vals[i][j]);
        }
        //y 
        for (int f = 0; f < s_f; ++f) 
          for (int r = 0; r < s_f; ++r) 
            for (int j = 0; j < sc0; ++j) {
              vars.add(matheus_model_4.y[i][f][r][j]);
              vals.add(matheus_model_4.y_vals[i][f][r][j]);
            }
      }
      setSolution(vars, vals, cost);
      //clean vals
      for (int i = 0; i < sc0; ++i) {
        matheus_model_4.x_vals[i].end();
        for (int f = 0; f < s_f; ++f) {
          for (int r = 0; r < s_f; ++r) 
            matheus_model_4.y_vals[i][f][r].end();
          matheus_model_4.y_vals[i][f].end();
        }
        matheus_model_4.y_vals[i].end();
      }
      matheus_model_4.x_vals.end();
      matheus_model_4.y_vals.end();
      t_vals.end();
      e_vals.end();
      vars.end();
      vals.end();
    } 
  } catch (IloException& e) {
    throw e;
  } catch (string& s) {
    throw s;
  } catch (...) {
    throw string("Error in setting MIP start solution");
  }
}
