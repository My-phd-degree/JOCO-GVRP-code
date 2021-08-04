#ifndef MIP_START_KK_MODEL_CPLEX_HPP_
#define MIP_START_KK_MODEL_CPLEX_HPP_

#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/cplex/kk_model/kk_model.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace kk_model {
        class Mip_start : public KK_model {
          public:
            explicit Mip_start (const Gvrp_instance& gvrp_instance, unsigned int time_limit, const Gvrp_solution& gvrp_solution); 
            const Gvrp_solution& gvrp_solution;
          protected:
            void extraStepsAfterModelCreation();
        };
      }
    }
  }
}

#endif
