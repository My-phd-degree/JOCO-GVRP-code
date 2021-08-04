#ifndef _GVRP_LOCAL_SEARCH_HPP_
#define _GVRP_LOCAL_SEARCH_HPP_

#include "models/local_search.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"

namespace models {
  namespace gvrp_models {
    class Gvrp_local_search : public Local_search<Gvrp_instance, Gvrp_solution> {
      public:
        explicit Gvrp_local_search (const Gvrp_instance& gvrp_instance, const Gvrp_solution& gvrp_solution);
        virtual ~Gvrp_local_search();
        virtual Gvrp_solution run() = 0;
    };
  }
}

#endif
