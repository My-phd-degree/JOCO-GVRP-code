#ifndef MIP_START_AFS_BOUNDS_CONSEC_CPLEX_HPP_
#define MIP_START_AFS_BOUNDS_CONSEC_CPLEX_HPP_

#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/objective_function_enum.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/afs_bounds_consec.hpp"

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace afs_bounds_consec {
        class Mip_start : public Afs_bounds_consec {
          public:
            explicit Mip_start (const Gvrp_instance& gvrp_instance, unsigned int time_limit, const Vertex& afs, const Gvrp_solution& gvrp_solution, Objective_function_enum objective_function_enum); 
            const Gvrp_solution& gvrp_solution;
          protected:
            void extraStepsAfterModelCreation();
        };
      }
    }
  }
}

#endif
