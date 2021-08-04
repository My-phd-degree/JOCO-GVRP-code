#ifndef DSU_HPP_
#define DSU_HPP_
#include <cstddef>

using namespace std;

namespace models {
  class DSU {
    public:
      unsigned int * pred, * rank;
      unsigned int n;
      explicit DSU (unsigned int n_);
      void join (unsigned int i, unsigned int j);
      unsigned int findSet (unsigned int i);
      void clean();
    private:
      void swap (unsigned int &a, unsigned int &b);
  };
}

#endif
