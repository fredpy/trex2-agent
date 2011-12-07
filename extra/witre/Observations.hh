#ifndef OBSERVATIONS
#define OBSERVATIONS

#include<string>

namespace TREX {
  namespace witre {

      class Observations {
        private:
            std::string observation;
            std::string object;
        public:
            Observations(std::string obs, std::string obj)
            :observation(obs), object(obj) {}
            std::string getObs() { return observation; };
            std::string getObj() { return object; };

      };
  }
}

#endif
