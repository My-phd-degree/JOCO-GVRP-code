#include "tests/gvrp/relaxation_tests.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/gvrp_feasible_solution_heuristic.hpp"
#include "models/cplex/mip_solution_info.hpp"
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/mip_start.hpp"
#include "models/gvrp_models/cplex/cubic_model/subcycle_user_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/invalid_edge_preprocessing.hpp"
#include "models/gvrp_models/cplex/cubic_model/invalid_edge_preprocessing_2.hpp"
#include "models/gvrp_models/cplex/cubic_model/invalid_edge_preprocessing_3.hpp"
#include "utils/util.hpp"
#include "SampleConfig.h"

#include <string>
#include <list>
#include <vector>

using namespace std;
using namespace utils;
using namespace tests::gvrp;
using namespace models::cplex;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::cubic_model;

Relaxation_tests::Relaxation_tests (bool VERBOSE_, unsigned int execution_time_, unsigned int nIntSol_): VERBOSE(VERBOSE_), execution_time(execution_time_), nIntSol(nIntSol_){}

void Relaxation_tests::run() {
  //experiments
  //setup
  string solution_name = "relaxed_cusbic_";
  string instance = "mlsa_flow_19c2f40.000000B-P-n22-k2.vrp";
  ofstream resultsFile;
  //model only
  resultsFile.open (solution_name + string("results.csv"));
  resultsFile<<"Instance;Cost;Time;Status;preprocessing1;preprocessing2;preprocessing3;preprocessing4"<<endl;
  Gvrp_instance gvrp_instance = matheus_instance_reader(PROJECT_INSTANCES_PATH + string("new/consec/") + instance);
  removeDistanceSymmetries (gvrp_instance.distances);
  cout<<gvrp_instance.distances[0][5]<<" "<<gvrp_instance.distances[5][0]<<endl;
  for (int i = 0; i < 100; ++i) {
    Cubic_model cubic_model (gvrp_instance, execution_time);  
    //keep this line, for some strange bug it is necessary to define the time limit explicitly 
    cubic_model.time_limit = execution_time;
    cubic_model.RELAXED = true;
    Mip_solution_info mipSolInfo;
    try {
      cubic_model.max_num_feasible_integer_sol = nIntSol;
      cubic_model.VERBOSE = VERBOSE;
      pair<Gvrp_solution, Mip_solution_info > sol = cubic_model.run();    
      mipSolInfo = sol.second;
    } catch (string s){
      cout<<s;
    } catch (const Mip_solution_info& excSolInfo){
      mipSolInfo = excSolInfo;
    } catch (...) {
      cout<<"Another error"<<endl;
    }
    resultsFile<<instance<<";"<<int(mipSolInfo.cost)<<"."<<int(mipSolInfo.cost*100)%100<<";"<<mipSolInfo.elapsed_time<<";"<<mipSolInfo.status<<";"<<cubic_model.nPreprocessings1<<";"<<cubic_model.nPreprocessings2<<";"<<cubic_model.nPreprocessings3<<";"<<cubic_model.nPreprocessings4<<endl;
  }
  resultsFile.close();
  resultsFile.clear();
}
