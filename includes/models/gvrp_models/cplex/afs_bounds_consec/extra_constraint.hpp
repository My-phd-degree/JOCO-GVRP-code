#ifndef EXTRA_CONSTRAINT_AFS_BOUNDS_CONSEC_CPLEX_HPP_
#define EXTRA_CONSTRAINT_AFS_BOUNDS_CONSEC_CPLEX_HPP_

#include "models/gvrp_models/cplex/afs_bounds_consec/afs_bounds_consec.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace afs_bounds_consec {
        class Afs_bounds_consec;
        class Extra_constraint {
          protected:
            Afs_bounds_consec& afs_bounds_consec;
          public:
            explicit Extra_constraint (Afs_bounds_consec& afs_bounds_consec);
            virtual ~Extra_constraint ();
            virtual void add() = 0; 
        };
      }
    }
  }
}

#endif
