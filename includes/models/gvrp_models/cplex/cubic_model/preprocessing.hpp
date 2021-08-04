#ifndef PREPROCESSING_CUBIC_MODEL_CPLEX_HPP_
#define PREPROCESSING_CUBIC_MODEL_CPLEX_HPP_

namespace models {
  namespace gvrp_models {
    namespace cplex {
      namespace cubic_model {
        class Cubic_model;
        class Preprocessing {
          protected:
            Cubic_model& cubic_model;
          public:
            explicit Preprocessing (Cubic_model& cubic_model);
            virtual ~Preprocessing ();
            virtual void add () = 0;
        };
      }
    }
  }
}

#endif
