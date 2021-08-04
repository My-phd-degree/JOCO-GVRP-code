#ifndef PREPROCESSING_AFS_BOUNDS_CONSEC_CPLEX_HPP_
#define PREPROCESSING_AFS_BOUNDS_CONSEC_CPLEX_HPP_

#include "models/gvrp_models/cplex/afs_bounds_consec/afs_bounds_consec.hpp" 

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace afs_bounds_consec {
        class Afs_bounds_consec;
        class Preprocessing {
          protected:
            Afs_bounds_consec& afs_bounds_consec;
          public:
            explicit Preprocessing (Afs_bounds_consec& afs_bounds_consec);
            virtual ~Preprocessing ();
            virtual void add () = 0;
        };
      }
    }
  }
}

#endif
