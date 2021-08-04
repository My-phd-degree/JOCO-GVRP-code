#ifndef PREPROCESSING_LH_MODEL_CPLEX_HPP_
#define PREPROCESSING_LH_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/lh_model/lh_model.hpp" 

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace lh_model {
        class LH_model;
        class Preprocessing {
          protected:
            LH_model& lh_model;
          public:
            explicit Preprocessing (LH_model& lh_model);
            virtual ~Preprocessing ();
            virtual void add () = 0;
        };
      }
    }
  }
}

#endif
