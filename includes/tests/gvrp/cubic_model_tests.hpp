#ifndef CUBIC_MODEL_TESTS_HPP_
#define CUBIC_MODEL_TESTS_HPP_

#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp" 

#include <string>

using namespace std;
using namespace models::gvrp_models::cplex::cubic_model;

namespace tests {
  namespace gvrp {
    class Cubic_model_tests {
      public:
        explicit Cubic_model_tests (bool VERBOSE, unsigned int execution_time, unsigned int nIntSol);
        void run ();
      private:
        bool VERBOSE;
        unsigned int execution_time;
        unsigned int nIntSol;
        void execute_model(Cubic_model& cubic_model, const string& instance_name, string& prefix_solution_files, unsigned int nIntSol, bool VERBOSE, Mip_solution_info& mipSolInfo);
        void openResultFile (ofstream& file, string fileName);
        void closeResultFile (ofstream& file);
    };
  }
}

#endif
