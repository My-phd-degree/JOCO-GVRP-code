#ifndef KK_MODEL_HPP_
#define KK_MODEL_HPP_

#include "models/vertex.hpp" 
#include "models/cplex/mip_solution_info.hpp" 
#include "models/gvrp_models/gvrp_instance.hpp" 
#include "models/gvrp_models/gvrp_afs_tree.hpp" 
#include "models/gvrp_models/gvrp_solution.hpp" 
#include "models/gvrp_models/cplex/gvrp_model.hpp" 

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
      namespace kk_model { 
        class KK_model : public Gvrp_model {
          public:
            explicit KK_model(const Gvrp_instance& gvrp_instance, unsigned int time_limit); 
            ~KK_model(); 
            pair<Gvrp_solution, Mip_solution_info> run();
            Matrix3DVar y;
            Matrix3DVal y_vals;
            Matrix2DVar x;
            Matrix2DVal x_vals;
            IloNumVarArray t;
            IloNumVarArray e;
            map<int, int> customersC0Indexes;
            map<int, int> afssF0Indexes;
            vector<const Vertex *> c0;
            vector<const Vertex *> f0;
            bool RELAXED;
            double time(int i, int f, int j) const;
            double time(int i, int j) const;
            double customersFuel(int i, int j) const;
            double customerToAfsFuel(int i, int f) const;
            double afsToCustomerFuel(int f, int i) const;
            double M1(int i, int f, int j);
            double M2(int i, int j);
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
