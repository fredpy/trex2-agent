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



                void setValue(TREX::transaction::goal_id const &g);

                /** Timeline Navigator */
                static TREX::utils::symbol const navigatorObj;
        };

    }


}

#endif //H_Rover
