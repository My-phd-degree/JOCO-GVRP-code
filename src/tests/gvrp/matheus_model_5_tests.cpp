#include "tests/gvrp/matheus_model_5_tests.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/matheus_model_5/matheus_model_5.hpp"
#include "utils/util.hpp"
#include "SampleConfig.h"

#include <string>
#include <list>
#include <iostream>
#include <float.h>

using namespace std;
using namespace utils;
using namespace tests::gvrp;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::matheus_model_5;

Matheus_model_5_tests::Matheus_model_5_tests (bool VERBOSE_, unsigned int execution_time_, unsigned int nIntSol_): VERBOSE(VERBOSE_), execution_time(execution_time_), nIntSol(nIntSol_){}

void Matheus_model_5_tests::run() {
  //experiments
    //setup
  string solution_name;
  Mip_solution_info mipSolInfo;
    //instance list
//  list<string> instances = listFilesFromDir (PROJECT_INSTANCES_PATH + string("EMH/"));
  list<string> instances = listFilesFromDir (PROJECT_INSTANCES_PATH + string("new/consec/"));
  list<Gvrp_instance> gvrp_instances;
  for (const string& instance : instances) {
//    Gvrp_instance gvrp_instance = erdogan_instance_reader(PROJECT_INSTANCES_PATH + string("EMH/") + instance);
    Gvrp_instance gvrp_instance = matheus_instance_reader(PROJECT_INSTANCES_PATH + string("new/consec/") + instance);
    gvrp_instances.push_back(gvrp_instance);
  }
    //executions
  auto gvrp_instance = gvrp_instances.begin();
  ofstream resultsFile;
      //model only
  solution_name = "matheus_model_5_";
  openResultFile(resultsFile, solution_name);
  int i = 0;
  for (const string& instance : instances) {
    cout<<instance<<endl;
    Matheus_model_5 matheus_model_5 (*gvrp_instance, execution_time);  
    execute_model(matheus_model_5, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
    resultsFile<<instance<<";"<<solution_name + instance<<";"<<mipSolInfo.gap<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<";"<<matheus_model_5.nPreprocessings1<<";"<<matheus_model_5.nPreprocessings2<<";"<<matheus_model_5.nPreprocessings3<<";"<<matheus_model_5.nPreprocessings4<<endl;
    ++gvrp_instance;
    ++i;
  }
  closeResultFile(resultsFile);
}

void Matheus_model_5_tests::execute_model(Matheus_model_5& matheus_model_5, const string& instance_name, string& prefix_solution_files, unsigned int nIntSol, bool VERBOSE, Mip_solution_info& mipSolInfo) {
  try {
    matheus_model_5.max_num_feasible_integer_sol = nIntSol;
    matheus_model_5.VERBOSE = VERBOSE;
    pair<Gvrp_solution, Mip_solution_info > sol = matheus_model_5.run();    
    sol.first.write_in_file(PROJECT_SOLUTIONS_PATH + string("MD_5/") + string(prefix_solution_files) + instance_name);
    mipSolInfo = sol.second;
  } catch (string s){
    cout<<"Error:"<<s;
  } catch (const Mip_solution_info& excSolInfo){
    mipSolInfo = excSolInfo;
  } catch (...) {
    cout<<"Another error"<<endl;
  }
}

void Matheus_model_5_tests::openResultFile (ofstream& resultsFile, string fileName) {
  resultsFile.open (fileName + string("md_results.csv"));
  resultsFile<<"Instance;Solution;GAP;Cost;Time;Status;preprocessing1;preprocessing2;preprocessing3;preprocessing4"<<endl;
}

void Matheus_model_5_tests::closeResultFile (ofstream& resultsFile) {
  resultsFile.close();
  resultsFile.clear();
}
