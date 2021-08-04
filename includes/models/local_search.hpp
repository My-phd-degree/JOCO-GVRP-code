#ifndef LOCAL_SEARCH_HPP_
#define LOCAL_SEARCH_HPP_

#include "models/local_search_strategy_enum.hpp"

namespace models {
  template <class I, class S> class Local_search {
    public:
      explicit Local_search(const I& instance_, const S& solution_) : instance (instance_), solution (solution_), newSolution (nullptr), strategy (FIRST_IMPROVEMENT) {} 
      virtual ~Local_search () {
        if (newSolution != nullptr)
          delete newSolution;
      }
      virtual S run() = 0;
      Local_search_strategy_enum strategy;
      const I& instance;
      const S& solution;
      S* newSolution;
  };
}

#endif
