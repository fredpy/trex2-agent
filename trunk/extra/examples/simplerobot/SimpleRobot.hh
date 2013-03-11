#ifndef H_Rover
#define H_Rover

#include <trex/transaction/TeleoReactor.hh>
#include <iostream>

namespace TREX {

    namespace simpleRobot {

        class SimpleRobot :public TREX::transaction::TeleoReactor
        {
            public:
                SimpleRobot(TREX::transaction::TeleoReactor::xml_arg_type arg);
                ~SimpleRobot();

            private:
                void handleInit();
                bool synchronize();
                void handleRequest(TREX::transaction::goal_id const &g);
                void handleRecall(TREX::transaction::goal_id const &g);

                /** @brief State of the timeline */
                std::string location;

                bool m_firstTick;
                TREX::transaction::TICK m_nextTick;

                void setValue(TREX::transaction::goal_id const &g);

                std::list<TREX::transaction::goal_id> m_pending;
                UNIQ_PTR<TREX::transaction::Observation> m_navigator_state;
                UNIQ_PTR<TREX::transaction::Observation> m_InstrumentLocation_state;
                UNIQ_PTR<TREX::transaction::Observation> m_Instrument_state;

                /** Timeline Navigator */
                static TREX::utils::Symbol const navigatorObj;
				static TREX::utils::Symbol const AtPred;
				static TREX::utils::Symbol const GoingPred;
				static TREX::utils::Symbol const locationPred;
				static TREX::utils::Symbol const pathPred;
        };

    }


}

#endif //H_Rover
