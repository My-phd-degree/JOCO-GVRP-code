#ifndef _BPP_INSTANCE_HPP_
#define _BPP_INSTANCE_HPP_

#include <sstream>
#include <vector>

using namespace std;

namespace models {
  namespace bpp_models {
    class BPP_instance {
      public:
        explicit BPP_instance(vector<double> items, double capacity);
        vector<double> items;
        double capacity;
        friend ostream& operator<<(ostream& strm, const BPP_instance& bpp_instance){
          stringstream output;
          output<<"Items:"<<endl;
          for (int i = 0; i < bpp_instance.items.size(); ++i) {
            output<<"\t"<<i<<": "<<bpp_instance.items[i]<<endl;
            output<<"Capacity: "<<bpp_instance.capacity<<endl;
            return strm << output.str();
          }
        }
    };
  }
}
#endif
