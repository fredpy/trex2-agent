#ifndef H_ROS_LISTENER
#define H_ROS_LISTENER

//#include "../../third_party/ros/trex/ros/ros_subscriber.hh"
//#include "../../third_party/ros/trex/ros/ros_client.hh"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <boost/asio/deadline_timer.hpp>

#include <trex/transaction/TeleoReactor.hh>
#include <iostream>
#include <list>
#include <mutex>

namespace TREX {

    namespace ecomapper {

        class Ros_Listener :public TREX::transaction::TeleoReactor
        {
            public:
                Ros_Listener(TREX::transaction::TeleoReactor::xml_arg_type arg);
                ~Ros_Listener();

                void latitudeCallback(const std_msgs::String::ConstPtr& msg);

            private:
                void handleInit();
                bool synchronize();
                void handleRequest(TREX::transaction::goal_id const &g);
                void handleRecall(TREX::transaction::goal_id const &g);

                bool started() const;
                void start();
                void stop() {
                    m_active = false;
                    m_freq.cancel();
                }
                void spin_cb();


                TREX::utils::SingletonUse<TREX::utils::LogManager> m_log;
                //TREX::utils::SingletonUse<TREX::ROS::ros_client> m_cli;
                ros::NodeHandle*                    m_ros;
                std::list<ros::Subscriber>         m_sub;

                std::list<TREX::transaction::Observation> obs;

                std::mutex Messagelock;

                mutable TREX::utils::SharedVar<bool> m_active;
                boost::asio::deadline_timer m_freq;

                TREX::transaction::TICK m_nextTick;

                std::list<TREX::transaction::goal_id> m_pending;

                static TREX::utils::Symbol const latitudeObj;
        };

    }


}

#endif //H_ROS_LISTENER

