#include "tests/gvrp/cubic_model_tests.hpp"
#include "tests/gvrp/emh_model_tests.hpp"
#include "tests/gvrp/kk_model_tests.hpp"
#include "tests/gvrp/lh_model_tests.hpp"
#include "tests/gvrp/matheus_model_4_tests.hpp"
#include "tests/gvrp/matheus_model_5_tests.hpp"
#include "tests/gvrp/relaxation_tests.hpp"
#include "tests/gvrp/afs_bounds_consec_tests.hpp"
#include "tests/gvrp/local_searchs_tests.hpp"
#include "tests/bpp/bpp_model_tests.hpp"

#include <string>

using namespace std;
using namespace tests::bpp;
using namespace tests::gvrp;

int main (int argc, char **argv)
{ 
  unsigned int execution_time = 120,
               nIntSol = -1;
  bool VERBOSE = false;  
  //getting params
  for (int i = 0; i < argc; i++)
    if (strcmp(argv[i], "-time") == 0)
      execution_time = stoi(argv[++i]);
    else if (strcmp(argv[i], "-verbose") == 0) {
      if (strcmp(argv[++i], "true") == 0 ||  strcmp(argv[i], "1") == 0) {
        VERBOSE = true; 
      }
    } else if (strcmp(argv[i], "-nIntSol") == 0) {
      nIntSol = stoi(argv[++i]);
      if (nIntSol <= 0)
        nIntSol = 2100000000;
    }
  //experiments
  /*
    //EMH model
  cout<<"============    EMH MODEL    =============="<<endl;
  EMH_model_tests emh_model_tests (VERBOSE, execution_time, nIntSol);
  emh_model_tests.run();
  */
    //KK model
  cout<<"============    KK MODEL    =============="<<endl;
  KK_model_tests kk_model_tests (VERBOSE, execution_time, nIntSol);
  kk_model_tests.run();
    //LH model
  cout<<"============    LH MODEL    =============="<<endl;
  LH_model_tests lh_model_tests (VERBOSE, execution_time, nIntSol);
  lh_model_tests.run();
  //cubic model
  cout<<"============    CUBIC MODEL    =============="<<endl;
  Cubic_model_tests cubic_model_tests (VERBOSE, execution_time, nIntSol);
  cubic_model_tests.run();
    //Matheus model 4 
  cout<<"============    MATHEUS 4 MODEL    =============="<<endl;
  Matheus_model_4_tests matheus_model_4_tests (VERBOSE, execution_time, nIntSol);
  matheus_model_4_tests.run();
    //Matheus model 5
  cout<<"============    MATHEUS 5 MODEL    =============="<<endl;
  Matheus_model_5_tests matheus_model_5_tests (VERBOSE, execution_time, nIntSol);
  matheus_model_5_tests.run();
  /*
    //Relaxation tests
  cout<<"============    RELAXATION TESTS    =============="<<endl;
  Relaxation_tests relaxation_tests (VERBOSE, execution_time, nIntSol);
  relaxation_tests.run();
    //AFS bounds consec 
  cout<<"============    AFS BOUNDS CONSEC MODEL    =============="<<endl;
  Afs_bounds_consec_tests afs_bounds_consec_tests (VERBOSE, execution_time, nIntSol);
  afs_bounds_consec_tests.run();
    //BBP model
  BPP_model_tests bpp_model_tests (VERBOSE, execution_time, nIntSol);
  bpp_model_tests.run();
  */
  return 0;
}
