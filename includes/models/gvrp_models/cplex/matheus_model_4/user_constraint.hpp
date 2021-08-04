#ifndef USER_CONSTRAINT_MATHEUS_MODEL_4_CPLEX_HPP_
#define USER_CONSTRAINT_MATHEUS_MODEL_4_CPLEX_HPP_

#include "models/gvrp_models/cplex/matheus_model_4/matheus_model_4.hpp" 

#include <ilcplex/ilocplex.h>

namespace models{
  namespace gvrp_models {
    namespace cplex {
      namespace matheus_model_4 {
        class Matheus_model_4;
        class User_constraint : public IloCplex::UserCutCallbackI {
          protected:
            Matheus_model_4& matheus_model_4;
            const double EPS;
          public:
            explicit User_constraint (Matheus_model_4& matheus_model_4);
        };    
      }
    }
  }
} 
#endif 
