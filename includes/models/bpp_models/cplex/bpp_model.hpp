#ifndef _BPP_MODEL_CPLEX_HPP_
#define _BPP_MODEL_CPLEX_HPP_

#include "models/cplex/mip_solution_info.hpp" 
#include "models/bpp_models/bpp_instance.hpp" 
#include "models/cplex/cplex_model.hpp" 

#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

using namespace std;
using namespace models::cplex;
using namespace models::bpp_models;

typedef vector<vector<int>> BPP_solution;
typedef IloArray<IloNumVarArray> Matrix2DVar;
typedef IloArray<IloNumArray> Matrix2DVal;

namespace models {
  namespace bpp_models {
    namespace cplex {
      class BPP_model : public Cplex_model<BPP_instance, BPP_solution> {
        public:
          explicit BPP_model(const BPP_instance& instance, unsigned int time_limit); 
          pair<BPP_solution, Mip_solution_info> run();
          const int sitems;
          IloNumVarArray y;
          Matrix2DVar x;
          IloNumArray y_vals;
          Matrix2DVal x_vals;
        protected:
          void createVariables();
          void createObjectiveFunction();
          void createModel();
          void fillVals();
          void createBPP_solution();
          void endVars();
      };
    }
  }
}

#endif
