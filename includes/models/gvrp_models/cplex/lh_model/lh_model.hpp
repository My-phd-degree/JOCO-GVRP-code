#ifndef LH_MODEL_HPP_
#define LH_MODEL_HPP_

#include "models/vertex.hpp" 
#include "models/cplex/mip_solution_info.hpp" 
#include "models/gvrp_models/gvrp_instance.hpp" 
#include "models/gvrp_models/gvrp_afs_tree.hpp" 
#include "models/gvrp_models/gvrp_solution.hpp" 
#include "models/gvrp_models/cplex/gvrp_model.hpp" 
#include "models/gvrp_models/cplex/lh_model/preprocessing.hpp"

#include <vector>
#include <map>
#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

typedef IloArray<IloNumVarArray> Matrix2DVar;
typedef IloArray<IloNumArray> Matrix2DVal;
typedef IloArray<Matrix2DVar> Matrix3DVar;
typedef IloArray<Matrix2DVal> Matrix3DVal;

using namespace std;
using namespace models;

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace lh_model { 
        class Preprocessing;
        class LH_model : public Gvrp_model {
          public:
            explicit LH_model(const Gvrp_instance& gvrp_instance, unsigned int time_limit); 
            ~LH_model();
            pair<Gvrp_solution, Mip_solution_info> run();
            Matrix3DVar y;
            Matrix2DVar x;
            Matrix2DVar a;
            Matrix2DVar u;
            Matrix2DVar v;
            Matrix3DVal y_vals;
            Matrix2DVal x_vals;
            list<Preprocessing*> preprocessings;
            map<int, int> customersC0Indexes;
            map<int, int> afssF0Indexes;
            vector<const Vertex *> c0;
            vector<const Vertex *> f0;
            int nPreprocessings0;
            int nPreprocessings1;
            int nPreprocessings2;
            int nPreprocessings3;
            bool RELAXED;
            double time(int i, int f, int j);
            double time(int i, int j);
            double customersFuel(int i, int j);
            double customerToAfsFuel(int i, int f);
            double afsToCustomerFuel(int f, int i);
          protected:
            void createVariables();
            void createObjectiveFunction();
            void createModel();
            virtual void extraStepsAfterModelCreation();
            void setCustomParameters();
            void fillVals();
            void createGvrp_solution();
            void endVals();
            void endVars();
        };
      }
    }
  }
}

#endif
