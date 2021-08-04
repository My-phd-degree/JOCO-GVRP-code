#ifndef LOCAL_SEARCHS_TESTS_HPP_
#define LOCAL_SEARCHS_TESTS_HPP_

#include "models/gvrp_models/local_searchs/merge.hpp"

#include <string>

using namespace std;
using namespace models::gvrp_models;
using namespace models::gvrp_models::local_searchs;

namespace tests {
  namespace gvrp {
    class Local_searchs_tests {
      public:
        void run ();
      private:
        void execute_model(Gvrp_local_search& local_search, const string& instance_name, string& prefix_solution_files);
        void openResultFile (ofstream& file, string fileName);
        void closeResultFile (ofstream& file);
    };
  }
}

#endif
