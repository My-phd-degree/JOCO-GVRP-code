#include "models/cplex/mip_depth.hpp"

#include <ilcplex/ilocplex.h>

using namespace models::cplex;

Depth::Depth(IloInt d) : depth(d) {}
