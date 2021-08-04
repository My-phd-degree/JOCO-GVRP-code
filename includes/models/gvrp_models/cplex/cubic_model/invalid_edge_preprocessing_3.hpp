#ifndef INVALID_EDGE_PREPROCESSING_3_CUBIC_MODEL_CPLEX_HPP_
#define INVALID_EDGE_PREPROCESSING_3_CUBIC_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/preprocessing.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace cubic_model {
        class Invalid_edge_preprocessing_3 : public Preprocessing {
          public:
            explicit Invalid_edge_preprocessing_3 (Cubic_model& cubic_model);
            void add ();
        };
      }
    }
  }
}

#endif
