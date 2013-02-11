#ifndef H_ECOMAPPER_NAVIGATOR
#define H_ECOMAPPER_NAVIGATOR

#include <trex/transaction/TeleoReactor.hh>
#include <iostream>
#include <mutex>
#include <boost/unordered_map.hpp>

//Open source library for Cubic Splines
#include "spline.hh"

namespace TREX {

    namespace ecomapper {

        class Navigator :public TREX::transaction::TeleoReactor
        {
            public:
                Navigator(TREX::transaction::TeleoReactor::xml_arg_type arg);
                ~Navigator();

            private:
                void handleInit();
                bool synchronize();
                void notify(TREX::transaction::Observation const &obs);
                void handleRequest(TREX::transaction::goal_id const &g);
                void handleRecall(TREX::transaction::goal_id const &g);

                void dvlObservation(TREX::transaction::Observation const &obs);
                void ctd_rhObservation(TREX::transaction::Observation const &obs);
                void fixObservation(TREX::transaction::Observation const &obs);
                void navSatFixObservation(TREX::transaction::Observation const &obs);

                double getAttribute(std::string const &name, TREX::transaction::Observation const &obs);

                void addToMap(std::string name, double value)
                    { values[name]=value; };
                double getMapValue(std::string name)
                    { return values[name];};

				//Cubic spline for path making
				magnet::math::Spline spline;
				std::mutex lock;

				boost::unordered_map<std::string, double> values;

                TREX::transaction::TICK m_nextTick;

                std::list<TREX::transaction::goal_id> m_pending;
        };

    }


}

#endif //H_ECOMAPPER_NAVIGATOR

