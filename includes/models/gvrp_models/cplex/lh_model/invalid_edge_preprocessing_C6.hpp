#ifndef INVALID_EDGE_PREPROCESSING_C6_LH_MODEL_CPLEX_HPP_
#define INVALID_EDGE_PREPROCESSING_C6_LH_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/lh_model/lh_model.hpp"
#include "models/gvrp_models/cplex/lh_model/preprocessing.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace lh_model {
        class Invalid_edge_preprocessing_C6 : public Preprocessing {
          public:
            explicit Invalid_edge_preprocessing_C6 (LH_model& lh_model);
            void add ();
        };
      }
    }
  }
}

#endif
