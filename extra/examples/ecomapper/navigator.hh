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

                void stateObservation(TREX::transaction::Observation const &obs);
                void dvlObservation(TREX::transaction::Observation const &obs);
                void wqmObservation(TREX::transaction::Observation const &obs);
                void ctd_rhObservation(TREX::transaction::Observation const &obs);
                void fixObservation(TREX::transaction::Observation const &obs);
                void navSatFixObservation(TREX::transaction::Observation const &obs);

                double getAttribute(std::string const &name, TREX::transaction::Observation const &obs);

                void addToMap(TREX::transaction::TICK tick, double value)
                    { columns[tick]=value; };
                double getMapValue(TREX::transaction::TICK tick)
                    { return columns[tick];};

				//Cubic spline for path making
				magnet::math::Spline spline;
				std::mutex lock;

                bool beginBoundaryTracking;
				bool ros_commanderBusy;

                typedef boost::unordered_map<TREX::transaction::TICK, double> TickDoubleMap;
				TickDoubleMap columns;
				typedef boost::unordered_map<TREX::transaction::TICK, std::pair<double, double> > CoordinateMap;
				CoordinateMap coordinates;
				TickDoubleMap wp_numbers;
				double nextwp_number;
				int numberOfSplinePoints;

                TREX::transaction::TICK currentTick;

                std::list<TREX::transaction::goal_id> m_pending;

                static TREX::utils::Symbol const navigatorObj;
                static TREX::utils::Symbol const waypointObj;
        };

    }


}

#endif //H_ECOMAPPER_NAVIGATOR

