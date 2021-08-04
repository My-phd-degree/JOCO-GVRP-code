#ifndef HEURISTIC_CALLBACK_AFS_BOUNDS_CONSEC_CPLEX_HPP_
#define HEURISTIC_CALLBACK_AFS_BOUNDS_CONSEC_CPLEX_HPP_

#include "models/gvrp_models/cplex/afs_bounds_consec/afs_bounds_consec.hpp" 

#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models;

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace afs_bounds_consec {
        class Heuristic_callback : public IloCplex::HeuristicCallbackI {
          protected:
            Afs_bounds_consec& afs_bounds_consec;
            const double EPS;
          public:
            explicit Heuristic_callback (Afs_bounds_consec& afs_bounds_consec);
        };
      } 
    } 
  } 
}
#endif
