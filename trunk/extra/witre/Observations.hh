#ifndef OBSERVATIONS
#define OBSERVATIONS

#include<string>

namespace TREX {
  namespace witre {

      class Observations {
        private:
            std::string observation;
            std::string object;
            std::string time;
        public:
            Observations(std::string obs, std::string obj, std::string tim="Tick")
            :observation(obs), object(obj), time(tim) {}
            std::string getObs() { return observation; };
            std::string getObj() { return object; };
            std::string getTime() { return time; };

      };
  }
}

#endif
