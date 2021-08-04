#ifndef PREPROCESSING_MATHEUS_MODEL_4_CPLEX_HPP_
#define PREPROCESSING_MATHEUS_MODEL_4_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model_4/matheus_model_4.hpp" 

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model_4 {
        class Matheus_model_4;
        class Preprocessing {
          protected:
            Matheus_model_4& matheus_model_4;
          public:
            explicit Preprocessing (Matheus_model_4& matheus_model_4);
            virtual ~Preprocessing ();
            virtual void add () = 0;
        };
      }
    }
  }
}

#endif
