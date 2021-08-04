#ifndef GVRP_LP_KK_HEURISTIC_HPP_
#define GVRP_LP_KK_HEURISTIC_HPP_

#include <vector>
#include <ilcplex/ilocplex.h>

#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/gvrp_heuristic.hpp"
#include "models/gvrp_models/cplex/kk_model/kk_model.hpp"

using namespace std;
using namespace models::gvrp_models::cplex::kk_model;

namespace models {
  namespace gvrp_models {
    class Gvrp_lp_kk_heuristic : public Gvrp_heuristic {
      public: 
        explicit Gvrp_lp_kk_heuristic (const KK_model& kk_model, const Matrix2DVal& x, const Matrix3DVal& y);
        const KK_model& kk_model;
        const Matrix2DVal& x;
        const Matrix3DVal& y;
        bool valid;
        Gvrp_solution run();
    };
  }
}

#endif
