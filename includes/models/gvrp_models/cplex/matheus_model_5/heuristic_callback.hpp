#ifndef HEURISTIC_CALLBACK_MATHEUS_MODEL_5_CPLEX_HPP_
#define HEURISTIC_CALLBACK_MATHEUS_MODEL_5_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model_5/matheus_model_5.hpp" 

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models;

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model_5 {
        class Heuristic_callback : public IloCplex::HeuristicCallbackI {
          protected:
            Matheus_model_5& matheus_model_5;
            const double EPS;
          public:
            explicit Heuristic_callback (Matheus_model_5& matheus_model_5);
        };
      } 
    } 
  } 
}
#endif
