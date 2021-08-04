#ifndef FSADD_GVRP_LOCAL_SEARCH_HPP_
#define FSADD_GVRP_LOCAL_SEARCH_HPP_

#include "models/gvrp_models/gvrp_local_search.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/local_search_strategy_enum.hpp"

namespace models {
  namespace gvrp_models {
    namespace local_searchs {
      class FSAdd : public Gvrp_local_search {
        public:
          explicit FSAdd(const Gvrp_instance& gvrp_instance, const Gvrp_solution& gvrp_solution, Local_search_strategy_enum strategy);
          ~FSAdd();
          Gvrp_solution run ();
      };
    }
  }
}

#endif
