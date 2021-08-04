#ifndef LAZY_CONSTRAINT_CUBIC_MODEL_CPLEX_HPP_
#define LAZY_CONSTRAINT_CUBIC_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"

#include <ilcplex/ilocplex.h>

namespace models{
  namespace gvrp_models {
    namespace cplex {
      namespace cubic_model {
        class Cubic_model;
        class Lazy_constraint : public IloCplex::LazyConstraintCallbackI {
          protected:
            Cubic_model& cubic_model;
          public:
            explicit Lazy_constraint (Cubic_model& cubic_model);
        };    
      }
    }
  }
} 
#endif 
