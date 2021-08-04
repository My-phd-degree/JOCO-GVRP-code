#ifndef SUBCYCLE_LAZY_CONSTRAINT_CUBIC_MODEL_CPLEX_HPP_
#define SUBCYCLE_LAZY_CONSTRAINT_CUBIC_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/cubic_model/lazy_constraint.hpp" 
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp" 

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models;

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace cubic_model {
        class Subcycle_lazy_constraint : public Lazy_constraint {
          public:
            Subcycle_lazy_constraint (Cubic_model& cubic_model);
            [[nodiscard]] IloCplex::CallbackI* duplicateCallback() const override;
            void main() override;
        };
      } 
    } 
  } 
}
#endif
