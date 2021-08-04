#ifndef SUBCYCLE_USER_CONSTRAINT_AFS_BOUNDS_CONSEC_CPLEX_HPP_
#define SUBCYCLE_USER_CONSTRAINT_AFS_BOUNDS_CONSEC_CPLEX_HPP_

#include "models/gvrp_models/cplex/afs_bounds_consec/user_constraint.hpp" 
#include "models/gvrp_models/cplex/afs_bounds_consec/afs_bounds_consec.hpp" 

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models;

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace afs_bounds_consec {
        class Subcycle_user_constraint : public User_constraint {
          public:
            Subcycle_user_constraint (Afs_bounds_consec& afs_bounds_consec);
            [[nodiscard]] IloCplex::CallbackI* duplicateCallback() const override;
            void main() override;
        };
      } 
    } 
  } 
}
#endif
