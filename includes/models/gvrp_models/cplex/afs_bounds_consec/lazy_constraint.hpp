#ifndef LAZY_CONSTRAINT_AFS_BOUNDS_CONSEC_CPLEX_HPP_
#define LAZY_CONSTRAINT_AFS_BOUNDS_CONSEC_CPLEX_HPP_

#include "models/gvrp_models/cplex/afs_bounds_consec/afs_bounds_consec.hpp" 

#include <ilcplex/ilocplex.h>

namespace models{
  namespace gvrp_models {
    namespace cplex {
      namespace afs_bounds_consec {
        class Afs_bounds_consec;
        class Lazy_constraint : public IloCplex::LazyConstraintCallbackI {
          protected:
            Afs_bounds_consec& afs_bounds_consec;
          public:
            explicit Lazy_constraint (Afs_bounds_consec& afs_bounds_consec);
        };    
      }
    }
  }
} 
#endif 
