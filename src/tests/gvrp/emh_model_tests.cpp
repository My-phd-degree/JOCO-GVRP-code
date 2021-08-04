#include "tests/gvrp/emh_model_tests.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/cplex/emh_model/emh_model.hpp"
#include "utils/util.hpp"
#include "SampleConfig.h"

#include <string>
#include <list>
#include <vector>
#include <iostream>

using namespace std;
using namespace utils;
using namespace tests::gvrp;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::emh_model;

EMH_model_tests::EMH_model_tests (bool VERBOSE_, unsigned int execution_time_, unsigned int nIntSol_): VERBOSE(VERBOSE_), execution_time(execution_time_), nIntSol(nIntSol_){}

void EMH_model_tests::run() {
  //experiments
    //setup
  string solution_name;
  Mip_solution_info mipSolInfo;
    //instance list
  list<string> instances = listFilesFromDir (PROJECT_INSTANCES_PATH + string("EMH/"));
 // list<string> instances = listFilesFromDir (PROJECT_INSTANCES_PATH + string("new/non-consec/"));
  list<Gvrp_instance> gvrp_instances;
  int i = 0;
  for (const string& instance : instances){
    Gvrp_instance gvrp_instance = erdogan_instance_reader(PROJECT_INSTANCES_PATH + string("EMH/") + instance);
 //   Gvrp_instance gvrp_instance = matheus_instance_reader(PROJECT_INSTANCES_PATH + string("new/non-consec/") + instance);
    gvrp_instances.push_back(gvrp_instance);
  }
    //executions
  auto gvrp_instance = gvrp_instances.begin();
  ofstream resultsFile;
      //model only
  solution_name = "emh_model_";
  openResultFile(resultsFile, solution_name);
  i = 0;
  for (const string& instance : instances) {
    cout<<instance<<endl;
    EMH_model emh_model (*gvrp_instance, execution_time);  
//    emh_model.RELAXED = true;
    execute_model(emh_model, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<endl;
    gvrp_instance++;
    i++;
  }
  closeResultFile(resultsFile);
}

void EMH_model_tests::execute_model(EMH_model& emh_model, const string& instance_name, string& prefix_solution_files, unsigned int nIntSol, bool VERBOSE, Mip_solution_info& mipSolInfo) {
  try {
    emh_model.max_num_feasible_integer_sol = nIntSol;
    emh_model.VERBOSE = VERBOSE;
    pair<Gvrp_solution, Mip_solution_info > sol = emh_model.run();    
    sol.first.write_in_file(PROJECT_SOLUTIONS_PATH + string("EMH/") + string(prefix_solution_files) + instance_name);
    mipSolInfo = sol.second;
  } catch (string s){
    cout<<"Error:"<<s;
  } catch (const Mip_solution_info& excSolInfo){
    mipSolInfo = excSolInfo;
  } catch (...) {
    cout<<"Another error"<<endl;
  }
}

void EMH_model_tests::openResultFile (ofstream& resultsFile, string fileName) {
  resultsFile.open (fileName + string("emh_results.csv"));
  resultsFile<<"Instance,Solution,GAP,Cost,Time,Status"<<endl;
}

void EMH_model_tests::closeResultFile (ofstream& resultsFile) {
  resultsFile.close();
  resultsFile.clear();
}
