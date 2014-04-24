#ifndef H_Rover
#define H_Rover

#include <trex/transaction/reactor.hh>
#include <iostream>

namespace TREX {

    namespace simpleRobot {

        class SimpleRobot :public TREX::transaction::reactor
        {
            public:
                SimpleRobot(TREX::transaction::reactor::xml_arg_type arg);
                ~SimpleRobot();

            private:
                void handle_init();
                bool synchronize();
                void handle_request(TREX::transaction::goal_id const &g);
                void handle_recall(TREX::transaction::goal_id const &g);



                void setValue(TREX::transaction::goal_id const &g);

                /** Timeline Navigator */
                static TREX::utils::symbol const navigatorObj;
        };

    }


}

#endif //H_Rover
