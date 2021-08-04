#ifndef INVALID_EDGE_PREPROCESSING_3_AFS_BOUNDS_CONSEC_CPLEX_HPP_
#define INVALID_EDGE_PREPROCESSING_3_AFS_BOUNDS_CONSEC_CPLEX_HPP_

#include "models/gvrp_models/cplex/afs_bounds_consec/afs_bounds_consec.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/preprocessing.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace afs_bounds_consec {
        class Invalid_edge_preprocessing_3 : public Preprocessing {
          public:
            explicit Invalid_edge_preprocessing_3 (Afs_bounds_consec& afs_bounds_consec);
            void add ();
        };
      }
    }
  }
}

#endif
