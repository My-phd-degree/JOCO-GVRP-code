#include "tests/bpp/bpp_model_tests.hpp"
#include "models/vertex.hpp"
#include "models/bpp_models/bpp_instance.hpp"
#include "models/cplex/mip_solution_info.hpp"
#include "models/bpp_models/cplex/bpp_model.hpp"
#include "utils/util.hpp"
#include "SampleConfig.h"

#include <string>
#include <list>
#include <vector>
#include <functional>
#include <limits.h>

using namespace std;
using namespace utils;
using namespace tests::bpp;
using namespace models::cplex;
using namespace models::bpp_models;
using namespace models::bpp_models::cplex;

BPP_model_tests::BPP_model_tests (bool VERBOSE_, unsigned int execution_time_, unsigned int nIntSol_): VERBOSE(VERBOSE_), execution_time(execution_time_), nIntSol(nIntSol_){}

void BPP_model_tests::run() {
  //experiments
    //setup
  string solution_name;
  Mip_solution_info mipSolInfo;
    //instance list
  list<string> instances = listFilesFromDir (PROJECT_INSTANCES_PATH + string("EMH/"));
  list<Gvrp_instance> gvrp_instances;
  int i = 0;
  for (const string& instance : instances){
    Gvrp_instance gvrp_instance = erdogan_instance_reader(PROJECT_INSTANCES_PATH + string("EMH/") + instance);
    gvrp_instances.push_back(gvrp_instance); 
    i++; 
  }
    //executions
  auto gvrp_instance = gvrp_instances.begin();
  ofstream resultsFile;
      //model only
  solution_name = "bpp_";
  openResultFile(resultsFile, solution_name);
  i = 0;
  for (const string& instance : instances) {
    //create bpp instance
    const int sc0 = gvrp_instance->customers.size() + 1;
    vector<double> minTimes(sc0, INT_MAX);
    vector<reference_wrapper<const Vertex>> c0;
    c0.push_back(gvrp_instance->depot);
    for (auto customer = gvrp_instance->customers.begin(); customer != gvrp_instance->customers.end(); ++customer)
      c0.push_back(*customer);
    for (int i = 0; i < sc0; ++i) {
      for (int j = 0; j < sc0; ++j)
        if (i != j) {
          minTimes[i] = min(minTimes[i], gvrp_instance->time(c0[i].get().id, c0[j].get().id));
        }
      minTimes[i] += c0[i].get().serviceTime;
    }
    BPP_instance bpp_instance (minTimes, gvrp_instance->timeLimit);
    //run model
    BPP_model bpp_model (bpp_instance, execution_time);  
    execute_model(bpp_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    //print test
    /*
    for (const vector<int>& bin : *bpp_model.solution) {
      for (int item : bin) {
        cout<<c0[item].get().id<<" ";
      }
      cout<<endl;
    }
    */
    //write in file
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    //next instance
    ++gvrp_instance;
    ++i;
  }
  closeResultFile(resultsFile);
}

void BPP_model_tests::execute_model(BPP_model& bpp_model, const string& instance_name, string& prefix_solution_files, unsigned int nIntSol, bool VERBOSE, Mip_solution_info& mipSolInfo) {
  try {
    bpp_model.max_num_feasible_integer_sol = nIntSol;
    bpp_model.VERBOSE = VERBOSE;
    pair<BPP_solution, Mip_solution_info > sol = bpp_model.run();    
    mipSolInfo = sol.second;
  } catch (string s){
    cout<<"Error:"<<s;
  } catch (const Mip_solution_info& excSolInfo){
    mipSolInfo = excSolInfo;
  } catch (...) {
    cout<<"Another error"<<endl;
  }
}

void BPP_model_tests::openResultFile (ofstream& resultsFile, string fileName) {
  resultsFile.open (fileName + string("results.csv"));
  resultsFile<<"Instance,Solution,GAP,Cost,Time,Status"<<endl;
}

void BPP_model_tests::closeResultFile (ofstream& resultsFile) {
  resultsFile.close();
  resultsFile.clear();
}
