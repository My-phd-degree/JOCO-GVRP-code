#ifndef BPP_MODEL_TESTS_HPP_
#define BPP_MODEL_TESTS_HPP_

#include "models/bpp_models/cplex/bpp_model.hpp" 

#include <string>

using namespace std;
using namespace models::bpp_models::cplex;

namespace tests {
  namespace bpp {
    class BPP_model_tests {
      public:
        explicit BPP_model_tests (bool VERBOSE, unsigned int execution_time, unsigned int nIntSol);
        void run ();
      private:
        bool VERBOSE;
        unsigned int execution_time;
        unsigned int nIntSol;
        void execute_model(BPP_model& bpp_model, const string& instance_name, string& prefix_solution_files, unsigned int nIntSol, bool VERBOSE, Mip_solution_info& mipSolInfo);
        void openResultFile (ofstream& file, string fileName);
        void closeResultFile (ofstream& file);
    };
  }
}

#endif
