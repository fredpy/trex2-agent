#ifndef H_Rover
#define H_Rover

#include <trex/transaction/TeleoReactor.hh>
#include <iostream>

namespace TREX {

    namespace rover {

        class Rover :public TREX::transaction::TeleoReactor
        {
            public:
                Rover(TREX::transaction::TeleoReactor::xml_arg_type arg);
                ~Rover();

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
                static TREX::utils::symbol const navigatorObj;
                    static TREX::utils::symbol const AtPred;
                    static TREX::utils::symbol const GoingPred;
                    static TREX::utils::symbol const locationPred;
                    static TREX::utils::symbol const pathPred;

                /** Timeline InstrumentLocation */
                static TREX::utils::symbol const instrumentLocationObj;
                    static TREX::utils::symbol const StowedPred;
                    static TREX::utils::symbol const StowingPred;
                    static TREX::utils::symbol const UnstowedPred;
                    static TREX::utils::symbol const UnstowingPred;

                /** Timline InstrumentState */
                static TREX::utils::symbol const instrumentStateObj;
                    static TREX::utils::symbol const PlacedPred;
                    static TREX::utils::symbol const SamplingPred;
                    static TREX::utils::symbol const FreePred;
        };

    }


}

#endif //H_Rover
