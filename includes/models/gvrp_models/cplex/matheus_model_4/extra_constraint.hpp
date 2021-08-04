#ifndef EXTRA_CONSTRAINT_MATHEUS_MODEL_4_CPLEX_HPP_
#define EXTRA_CONSTRAINT_MATHEUS_MODEL_4_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model_4/matheus_model_4.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model_4 {
        class Matheus_model_4;
        class Extra_constraint {
          protected:
            Matheus_model_4& matheus_model_4;
          public:
            explicit Extra_constraint (Matheus_model_4& matheus_model_4);
            virtual ~Extra_constraint ();
            virtual void add() = 0; 
        };
      }
    }
  }
}

#endif
