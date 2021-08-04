#ifndef UTIL_HPP_
#define UTIL_HPP_

#include "models/vertex.hpp"
#include "models/vrp_instance.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/gvrp_afs_tree.hpp"
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"

#include <string>
#include <list>
#include <map>
#include <vector>

using namespace models;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex::cubic_model;
using namespace std;

namespace utils {
  //gvrp
  pair<vector<vector<double>>, vector<vector<double>>> calculateGVRPReducedGraphs (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree);
  vector<pair<double, double>> calculateClosestsGVRPCustomers (const vector<vector<double>>& gvrpReducedGraph, const vector<const Vertex *>& vertices);
  int calculateGVRP_BPP_NRoutesLB(const Gvrp_instance& gvrp_instance, const vector<const Vertex *>& vertices, const vector<pair<double, double>>& closestsTimes, unsigned int execution_time_limit);
  double calculateGvrpInstanceLambda (const Gvrp_instance& gvrp_instance);
  double calculateGvrpInstancePsi (const Gvrp_instance& gvrp_instance);
  double calculateCustomerMinRequiredFuel (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree, const Vertex& customer);
  double calculateCustomerMinRequiredTime (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree, const Vertex& customer);
  double calculateCustomerMaxAllowedFuel (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree, const Vertex& customer);
  double calculateGvrpLBByImprovedMST (const vector<const Vertex *>& vertices, const vector<pair<double, double>> closests, const vector<vector<double>>& gvrpReducedGraph);
  double calculateGvrpLBByImprovedMSTTime (const vector<const Vertex *>& vertices, const vector<pair<double, double>> closestsTimes, const vector<vector<double>>& gvrpReducedGraphTimes);
  list<list<Vertex> > getGvrpConnectedComponents (const Gvrp_instance& gvrp_instance);
  list<pair<int, int>> get_invalid_edges_1 (const Gvrp_instance& gvrp_instance);
  list<pair<int, int>> get_invalid_edges_2 (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree);
  list<pair<int, int>> get_invalid_edges_3 (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree);
  list<pair<int, int>> get_invalid_edges_4 (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree);
  double calculateGvrpLB1 (const vector<pair<double, double>>& closestsDistances);
  double calculateGvrpBestLB (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree);
  double calculateGvrpBestNRoutesLB (const Gvrp_instance& gvrp_instance, const Gvrp_afs_tree& gvrp_afs_tree);
  //reading
  Gvrp_instance matheus_instance_reader(const string& file_path);
  Vrp_instance read_uchoa_vrp_instance (const string &file_path);    
//  vector<vector<int>> read_uchoa_vrp_solution (const string &file_path);    
  Gvrp_instance erdogan_instance_reader(const string file_path);
  Gvrp_instance andelmin_bartolini_instance_reader(const string& file_path);
  Vrp_instance read_augerat_vrp_1995_setP_instance (const string& file_path);
  list<string> listFilesFromDir(string path);
  Gvrp_solution read_gvrp_solution (const string& file_path, const Gvrp_instance& gvrp_instance);
  void removeDistanceSymmetries (vector<vector<double>>& distances);
} 
#endif
