#ifndef AFS_BOUNDS_CONSEC_TESTS_HPP_
#define AFS_BOUNDS_CONSEC_TESTS_HPP_

#include "models/gvrp_models/cplex/afs_bounds_consec/afs_bounds_consec.hpp"

#include <string>

using namespace std;
using namespace models::gvrp_models::cplex::afs_bounds_consec;

namespace tests {
  namespace gvrp {
    class Afs_bounds_consec_tests {
      public:
        explicit Afs_bounds_consec_tests (bool VERBOSE, unsigned int execution_time, unsigned int nIntSol);
        void run ();
      private:
        bool VERBOSE;
        unsigned int execution_time;
        unsigned int nIntSol;
        void execute_model(Afs_bounds_consec& afs_bounds_consec, const string& instance_name, string& prefix_solution_files, unsigned int nIntSol, bool VERBOSE, Mip_solution_info& mipSolInfo);
        void openResultFile (ofstream& file, string fileName);
        void closeResultFile (ofstream& file);
    };
  }
}

#endif
