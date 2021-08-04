#ifndef MIP_DEPTH_CPLEX_HPP_
#define MIP_DEPTH_CPLEX_HPP_

#include <ilcplex/ilocplex.h>

namespace models {
  namespace cplex {
    class Depth : public IloCplex::MIPCallbackI::NodeData {
      public:
        explicit Depth(IloInt d); 
        IloInt const depth;
    };
  }
}

#endif
