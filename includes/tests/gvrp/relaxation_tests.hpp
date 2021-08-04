#ifndef RELAXATION_TESTS_HPP_
#define RELAXATION_TESTS_HPP_

#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp" 

#include <string>

using namespace std;
using namespace models::gvrp_models::cplex::cubic_model;

namespace tests {
  namespace gvrp {
    class Relaxation_tests {
      public:
        explicit Relaxation_tests (bool VERBOSE, unsigned int execution_time, unsigned int nIntSol);
        void run ();
      private:
        bool VERBOSE;
        unsigned int execution_time;
        unsigned int nIntSol;
    };
  }
}

#endif
