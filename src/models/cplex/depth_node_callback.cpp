#include "models/cplex/depth_node_callback.hpp"
#include "models/cplex/mip_depth.hpp"

#include <ilcplex/ilocplex.h>

using namespace models::cplex;

Depth_node_callback::Depth_node_callback (IloEnv env) : IloCplex::NodeCallbackI (env) {}

void Depth_node_callback::main () {
  IloInt64 nextNode = 0;
  Depth *depth = new Depth(getDepth(nextNode));
  NodeData *old = setNodeData(nextNode, depth); // Replace node data in nextNode
  if ( old )  delete old;                       // Delete old node data (if there was any
}

[[nodiscard]] IloCplex::CallbackI* Depth_node_callback::duplicateCallback() const {
  return new (getEnv()) Depth_node_callback (*this);
} 
