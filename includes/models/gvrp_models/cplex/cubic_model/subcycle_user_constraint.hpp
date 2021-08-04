#ifndef SUBCYCLE_USER_CONSTRAINT_CUBIC_MODEL_CPLEX_HPP_
#define SUBCYCLE_USER_CONSTRAINT_CUBIC_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/cubic_model/user_constraint.hpp" 
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp" 

#include <vector>
#include <unordered_set>
#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models;

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace cubic_model {
        class Subcycle_user_constraint : public User_constraint {
          public:
            Subcycle_user_constraint (Cubic_model& cubic_model);
            [[nodiscard]] IloCplex::CallbackI* duplicateCallback() const override;
            void main() override;
        };
      } 
    } 
  } 
}
#endif
