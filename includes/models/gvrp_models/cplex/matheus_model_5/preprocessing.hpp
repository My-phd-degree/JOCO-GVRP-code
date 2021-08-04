#ifndef PREPROCESSING_MATHEUS_MODEL_5_CPLEX_HPP_
#define PREPROCESSING_MATHEUS_MODEL_5_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model_5/matheus_model_5.hpp" 

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model_5 {
        class Matheus_model_5;
        class Preprocessing {
          protected:
            Matheus_model_5& matheus_model_5;
          public:
            explicit Preprocessing (Matheus_model_5& matheus_model_5);
            virtual ~Preprocessing ();
            virtual void add () = 0;
        };
      }
    }
  }
}

#endif
