#include "models/cplex/mip_solution_info.hpp"
#include "models/bpp_models/bpp_instance.hpp"
#include "models/bpp_models/cplex/bpp_model.hpp"

#include <sstream>
#include <list>
#include <time.h> 
#include <string> 

using namespace models::cplex;
using namespace models::bpp_models;
using namespace models::bpp_models::cplex;

using namespace std;

BPP_model::BPP_model(const BPP_instance& instance, unsigned int time_limit) : Cplex_model(instance, time_limit), sitems(instance.items.size()) {

} 

pair<BPP_solution, Mip_solution_info> BPP_model::run(){
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
    cplex.setOut(env.getNullStream());
//    cout<<"Solving model"<<endl;
    if ( !cplex.solve() ) {
//      env.error() << "Failed to optimize LP." << endl;
      mipSolInfo = Mip_solution_info(-1, cplex.getStatus(), -1, -1);
      endVars();
//      env.end();
      throw mipSolInfo;
    }
//    cplex.exportModel("cplexcpp.lp");
//    env.out() << "Solution value = " << cplex.getObjValue() << endl;
//    cout<<"Getting x values"<<endl;
    fillVals();
//    cout<<"Creating GVRP solution"<<endl;
    createBPP_solution();
    mipSolInfo = Mip_solution_info(cplex.getMIPRelativeGap(), cplex.getStatus(), cplex.getTime(), cplex.getObjValue());
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

void BPP_model::createVariables(){
  try {
    x = Matrix2DVar (env, sitems);
    y = IloNumVarArray (env, sitems, 0, 1, IloNumVar::Int);
    //setting names
    stringstream nameStream;
    for (int i = 0; i < sitems; ++i){
      //y var
      nameStream<<"y["<<i<<"]";
      y[i].setName(nameStream.str().c_str());
      nameStream.clear();
      nameStream.str("");
      //x var
      x[i] = IloNumVarArray(env, sitems, 0, 1, IloNumVar::Int);
      for (int j = 0; j < sitems; ++j){
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

void BPP_model::createObjectiveFunction() {
  //objective function
  try{
    IloExpr fo (env);
    for (int j = 0; j < sitems; ++j)
      fo += y[j];
    model = IloModel (env);
    model.add(IloMinimize(env, fo));
  } catch (IloException& e) {
    throw e;
  } catch(...){
    throw string("Error in creating the objective function");
  }
}

void BPP_model::createModel() {
  try {
    //constraints
    IloExpr expr(env);
    IloConstraint c;
    stringstream constraintName;
    //\sum_{j \in B} x_{ij} = 1, \forall v_i \in I
    for (int i = 0; i < sitems; ++i) {
      for (int j = 0; j < sitems; ++j)
        expr += x[i][j];
      c = IloConstraint (expr == 1);
      constraintName<<"item "<<i<<" must be in exactly one bin";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{i \in I} x_{ij} p_i \leqslant C y_j, \forall j \in B
    for (int j = 0; j < sitems; ++j) {
      for (int i = 0; i < sitems; ++i)
          expr += x[i][j] * instance.items[i];
      c = IloConstraint (expr <= instance.capacity * y[j]);
      constraintName<<"bin "<<j<<" consumed  must be less than "<<instance.capacity<<" if bin is openned";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{i \in I} x_{ij} p_i \leqslant C y_j, \forall j \in B
    for (int j = 0; j < sitems; ++j) {
      for (int i = 0; i < sitems; ++i) {
        c = IloConstraint (x[i][j] <= y[j]);
        constraintName<<"item "<<i<<" only is placed at bin "<<j<<" if bin "<<j<<" is openned";
        c.setName(constraintName.str().c_str());
        model.add(c);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    }
    //y_j \leqslant y_{j - 1}, \forall j \in \{1, ..., |B|\} \subset B
    for (int j = 1; j < sitems; ++j) {
      c = IloConstraint (y[j] <= y[j - 1]);
      constraintName<<"bin "<<j<<" only will be openned if bin "<<j - 1<<" is openned";
      c.setName(constraintName.str().c_str());
      model.add(c);
      constraintName.clear();
      constraintName.str("");
    }
    //y_0 = 1
    c = IloConstraint (y[0] == 1);
    constraintName<<"bin "<<0<<" will be openned";
    c.setName(constraintName.str().c_str());
    model.add(c);
    constraintName.clear();
    constraintName.str("");
    //init
    cplex = IloCplex(model);
  } catch (IloException& e) {
    throw e;
  } catch (string s) {
    throw s;
  }
}

void BPP_model::fillVals(){
  //getresult
  try{
    x_vals = Matrix2DVal (env, sitems);
    y_vals = IloNumArray (env, sitems);
    cplex.getValues(y_vals, y);
    for (int i = 0; i < sitems; ++i) {
      x_vals[i] = IloNumArray (env, sitems, 0, 1, IloNumVar::Int);
      cplex.getValues(x_vals[i], x[i]);
    }
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting solution");
  }
}

void BPP_model::createBPP_solution(){
  try{
    solution = new BPP_solution ();
    for (int j = 0; j < sitems; ++j) {
      if (y_vals[j] > 0) {
        vector<int> bin;
        for (int i = 0; i < sitems; ++i) 
          if (x_vals[i][j] > 0)
            bin.push_back(i);
        solution->push_back(bin);
      }
    }
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting routes");
  }
}

void BPP_model::endVars(){
  for (int i = 0; i < sitems; ++i) 
    x[i].end();
  y.end();
  x.end(); 
}
