#ifndef GREEDY_LP_HEURISTIC_CUBIC_MODEL_CPLEX_HPP_
#define GREEDY_LP_HEURISTIC_CUBIC_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp" 
#include "models/gvrp_models/cplex/cubic_model/heuristic_callback.hpp" 

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models;

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace cubic_model {
        class Greedy_lp_heuristic : public Heuristic_callback {
          public:
            Greedy_lp_heuristic (Cubic_model& cubic_model);
            [[nodiscard]] IloCplex::CallbackI* duplicateCallback() const override;
            void main() override;
        };
      } 
    } 
  } 
}
#endif
