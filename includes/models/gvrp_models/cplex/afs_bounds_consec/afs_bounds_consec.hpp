#ifndef _AFS_BOUNDS_CONSEC_HPP_
#define _AFS_BOUNDS_CONSEC_HPP_

#include "models/vertex.hpp" 
#include "models/objective_function_enum.hpp" 
#include "models/cplex/mip_solution_info.hpp" 
#include "models/gvrp_models/gvrp_instance.hpp" 
#include "models/gvrp_models/gvrp_afs_tree.hpp" 
#include "models/gvrp_models/gvrp_solution.hpp" 
#include "models/gvrp_models/cplex/gvrp_model.hpp" 
#include "models/gvrp_models/cplex/afs_bounds_consec/user_constraint.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/lazy_constraint.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/heuristic_callback.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/preprocessing.hpp"
#include "models/gvrp_models/cplex/afs_bounds_consec/extra_constraint.hpp"

#include <vector>
#include <map>
#include <unordered_set>
#include <ilcplex/ilocplex.h>

ILOSTLBEGIN

typedef IloArray<IloNumVarArray> Matrix2DVar;
typedef IloArray<IloNumArray> Matrix2DVal;
typedef IloArray<Matrix2DVar> Matrix3DVar;
typedef IloArray<Matrix3DVar> Matrix4DVar;
typedef IloArray<Matrix2DVal> Matrix3DVal;
typedef IloArray<Matrix3DVal> Matrix4DVal;

using namespace std;
using namespace models;

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace afs_bounds_consec { 
        class User_constraint;
        class Heuristic_callback;
        class Lazy_constraint;
        class Preprocessing;
        class Extra_constraint;
        class Afs_bounds_consec : public Gvrp_model {
          public:
            explicit Afs_bounds_consec(const Gvrp_instance& gvrp_instance, unsigned int time_limit, const Vertex& afs, Objective_function_enum objective_function_enum); 
            ~Afs_bounds_consec(); 
            pair<Gvrp_solution, Mip_solution_info> run();
            const Vertex& afs;
            Matrix4DVar y;
            Matrix2DVar x;
            Matrix2DVar a;
            Matrix2DVar u;
            Matrix2DVar v;
            Matrix2DVar e;
            Matrix4DVal y_vals;
            Matrix2DVal x_vals;
            list<User_constraint*> user_constraints;
            list<Lazy_constraint*> lazy_constraints;
            list<Heuristic_callback*> heuristic_callbacks;
            list<Preprocessing*> preprocessings;
            list<Extra_constraint*> extra_constraints;
            map<int, int> customersC0Indexes;
            map<int, int> afss_FIndexes;
            vector<const Vertex *> c0;
            vector<const Vertex *> _f;
            vector<double> customersMinRequiredFuel;
            vector<double> customersMinRequiredTime;
            vector<vector<double>> gvrpReducedGraphDistances;
            vector<vector<double>> gvrpReducedGraphTimes;
            unsigned long int nGreedyLP;
            unsigned int BPPTimeLimit;
            long int levelSubcycleCallback;
            int nRoutesLB;
            int nPreprocessings1;
            int nPreprocessings2;
            int nPreprocessings3;
            int nPreprocessings4;
            int nImprovedMSTNRoutesLB;
            int nBPPNRoutesLB;
            double solLB;
            bool RELAXED;
            Objective_function_enum objective_function_enum;
            double time(int i, int f, int r, int j);
            double time(int i, int j);
            double customersFuel(int i, int j);
            double customerToAfsFuel(int i, int f);
            double afsToCustomerFuel(int f, int i);
            double distance (int i, int f, int r, int j);
            list<Vertex> getAFSsShortestPath (const Vertex& ori, const Vertex& dest);
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
