#ifndef HEURISTIC_CALLBACK_CUBIC_MODEL_CPLEX_HPP_
#define HEURISTIC_CALLBACK_CUBIC_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp" 

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models;

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace cubic_model {
        class Heuristic_callback : public IloCplex::HeuristicCallbackI {
          protected:
            Cubic_model& cubic_model;
            const double EPS;
          public:
            explicit Heuristic_callback (Cubic_model& cubic_model);
        };
      } 
    } 
  } 
}
#endif
