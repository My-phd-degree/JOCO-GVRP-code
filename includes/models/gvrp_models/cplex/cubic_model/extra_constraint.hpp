#ifndef EXTRA_CONSTRAINT_CUBIC_MODEL_CPLEX_HPP_
#define EXTRA_CONSTRAINT_CUBIC_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace cubic_model {
        class Cubic_model;
        class Extra_constraint {
          protected:
            Cubic_model& cubic_model;
          public:
            explicit Extra_constraint (Cubic_model& cubic_model);
            virtual ~Extra_constraint ();
            virtual void add() = 0; 
        };
      }
    }
  }
}

#endif
