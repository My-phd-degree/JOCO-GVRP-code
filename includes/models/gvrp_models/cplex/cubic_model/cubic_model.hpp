#ifndef CUBIC_MODEL_HPP_
#define CUBIC_MODEL_HPP_

#include "models/cplex/mip_solution_info.hpp"
#include "models/cplex/cplex_model.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/cplex/gvrp_model.hpp"
#include "models/gvrp_models/gvrp_afs_tree.hpp"
#include "models/gvrp_models/cplex/cubic_model/lazy_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/user_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/cubic_model/extra_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/heuristic_callback.hpp"

#include <unordered_map>
#include <unordered_set>
#include <list>
#include <ilcplex/ilocplex.h>

ILOSTLBEGIN

using namespace std;
using namespace models::cplex;

typedef IloArray<IloArray<IloNumVarArray> > Matrix3DVar;
typedef IloArray<IloArray<IloNumArray> > Matrix3DVal;

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace cubic_model {
        class Lazy_constraint;
        class User_constraint;
        class Preprocessing;
        class Extra_constraint;
        class Cubic_model : public Gvrp_model {
          public:
            Cubic_model(const Gvrp_instance& gvrp_instance, unsigned int time_limit); 
            ~Cubic_model(); 
            pair<Gvrp_solution, Mip_solution_info> run();
            IDVertex all;
            unordered_set<int> afss;
            Matrix3DVar x;
            IloNumVarArray e;
            list<Lazy_constraint*> lazy_constraints;
            list<User_constraint*> user_constraints;
            list<Preprocessing*> preprocessings;
            list<Extra_constraint*> extra_constraints;
            list<Heuristic_callback*> heuristic_callbacks;
            vector<vector<double>> gvrpReducedGraphTimes;
            unordered_map<int,double> customersMinRequiredFuel;
            unsigned long int nGreedyLP;
            unsigned int BPPTimeLimit;
            long int levelSubcycleCallback;
            int nPreprocessings1;
            int nPreprocessings2;
            int nPreprocessings3;
            int nPreprocessings4;
            int nImprovedMSTNRoutesLB;
            int nBPPNRoutesLB;
            int nImprovedMSTNRoutesLBLazy;
            int nBPPNRoutesLBLazy;
            int nRoutesLB;
            double solLB;
            Matrix3DVal x_vals;
            bool RELAXED;
          protected:
            void createVariables();
            void createObjectiveFunction();
            void createModel();
            virtual void extraStepsAfterModelCreation();
            void setCustomParameters();
            void fillX_vals();
            void createGvrp_solution();
            void endVals();
            void endVars();
        };
      } 
    } 
  } 
}
#endif
