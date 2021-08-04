#ifndef LAZY_CONSTRAINT_MATHEUS_MODEL_4_CPLEX_HPP_
#define LAZY_CONSTRAINT_MATHEUS_MODEL_4_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model_4/matheus_model_4.hpp" 

#include <ilcplex/ilocplex.h>

namespace models{
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model_4 {
        class Matheus_model_4;
        class Lazy_constraint : public IloCplex::LazyConstraintCallbackI {
          protected:
            Matheus_model_4& matheus_model_4;
          public:
            explicit Lazy_constraint (Matheus_model_4& matheus_model_4);
        };    
      }
    }
  }
} 
#endif 
