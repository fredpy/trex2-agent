#ifndef H_ECOMAPPER_NAVIGATOR
#define H_ECOMAPPER_NAVIGATOR

#include <trex/transaction/TeleoReactor.hh>
#include <iostream>

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


                TREX::transaction::TICK m_nextTick;

                std::list<TREX::transaction::goal_id> m_pending;
        };

    }


}

#endif //H_ECOMAPPER_NAVIGATOR

