#include "models/objective_function_enum.hpp" 
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/afs_bounds_consec.hpp"
#include "tests/gvrp/afs_bounds_consec_tests.hpp"
#include "utils/util.hpp"
#include "SampleConfig.h"

#include <string>
#include <list>
#include <iostream>
#include <float.h>
#include <time.h> 

using namespace std;
using namespace utils;
using namespace tests::gvrp;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::afs_bounds_consec;

Afs_bounds_consec_tests::Afs_bounds_consec_tests (bool VERBOSE_, unsigned int execution_time_, unsigned int nIntSol_): VERBOSE(VERBOSE_), execution_time(execution_time_), nIntSol(nIntSol_){}

void Afs_bounds_consec_tests::run() {
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
  solution_name = "afs_bounds_consec_";
  openResultFile(resultsFile, solution_name);
  int i = 0;
  for (const string& instance : instances) {
    cout<<instance<<endl;
//    Gvrp_feasible_solution_heuristic gfsh (*gvrp_instance);
//    Gvrp_solution gvrp_solution = gfsh.run();
//    Mip_start afs_bounds_consec (*gvrp_instance, execution_time, gvrp_solution);  
    int nDummies = 0;
    struct timespec start, finish;
    double elapsed;
    clock_gettime(CLOCK_MONOTONIC, &start);
    for (const Vertex& afs : gvrp_instance->afss) {
      Afs_bounds_consec afs_bounds_consec (*gvrp_instance, execution_time, afs, MAX);  
      afs_bounds_consec.RELAXED = true;
      execute_model(afs_bounds_consec, instance, solution_name, nIntSol, VERBOSE, mipSolInfo);
      nDummies += int(mipSolInfo.cost);
    }
    clock_gettime(CLOCK_MONOTONIC, &finish);
    elapsed = (finish.tv_sec - start.tv_sec) + (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
    resultsFile<<instance<<";"<<solution_name + instance<<";"<<nDummies<<";"<<elapsed<<endl;
    ++gvrp_instance;
    ++i;
  }
  closeResultFile(resultsFile);
}

void Afs_bounds_consec_tests::execute_model(Afs_bounds_consec& afs_bounds_consec, const string& instance_name, string& prefix_solution_files, unsigned int nIntSol, bool VERBOSE, Mip_solution_info& mipSolInfo) {
  try {
    afs_bounds_consec.max_num_feasible_integer_sol = nIntSol;
    afs_bounds_consec.VERBOSE = VERBOSE;
    pair<Gvrp_solution, Mip_solution_info > sol = afs_bounds_consec.run();    
    sol.first.write_in_file(PROJECT_SOLUTIONS_PATH + string("AFS_BOUNDS_CONSEC/") + string(prefix_solution_files) + instance_name);
    mipSolInfo = sol.second;
  } catch (string s){
    cout<<"Error:"<<s;
  } catch (const Mip_solution_info& excSolInfo){
    mipSolInfo = excSolInfo;
  } catch (...) {
    cout<<"Another error"<<endl;
  }
}

void Afs_bounds_consec_tests::openResultFile (ofstream& resultsFile, string fileName) {
  resultsFile.open (fileName + string("afs_bounds_consec_results.csv"));
  resultsFile<<"Instance;Solution;Cost;Time"<<endl;
}

void Afs_bounds_consec_tests::closeResultFile (ofstream& resultsFile) {
  resultsFile.close();
  resultsFile.clear();
}
