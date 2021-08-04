#ifndef USER_CONSTRAINT_MATHEUS_MODEL_5_CPLEX_HPP_
#define USER_CONSTRAINT_MATHEUS_MODEL_5_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model_5/matheus_model_5.hpp" 

#include <ilcplex/ilocplex.h>

namespace models{
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model_5 {
        class Matheus_model_5;
        class User_constraint : public IloCplex::UserCutCallbackI {
          protected:
            Matheus_model_5& matheus_model_5;
            const double EPS;
          public:
            explicit User_constraint (Matheus_model_5& matheus_model_5);
        };    
      }
    }
  }
} 
#endif 
