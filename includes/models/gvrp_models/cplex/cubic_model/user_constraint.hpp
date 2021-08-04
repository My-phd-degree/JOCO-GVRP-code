#ifndef USER_CONSTRAINT_CUBIC_MODEL_CPLEX_HPP_
#define USER_CONSTRAINT_CUBIC_MODEL_CPLEX_HPP_

#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp" 

#include <ilcplex/ilocplex.h>

namespace models{
  namespace gvrp_models {
    namespace cplex {
      namespace cubic_model {
        class Cubic_model;
        class User_constraint : public IloCplex::UserCutCallbackI {
          protected:
            Cubic_model& cubic_model;
            const double EPS;
          public:
            explicit User_constraint (Cubic_model& cubic_model);
        };    
      }
    }
  }
} 
#endif 
