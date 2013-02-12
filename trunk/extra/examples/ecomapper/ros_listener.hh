#ifndef H_ROS_LISTENER
#define H_ROS_LISTENER

#include "ros/ros.h"
#include "std_msgs/String.h"

//ecomapper message types
#include "ecomapper_msgs/DVL.h"
#include "ecomapper_msgs/CTD.h"
#include "ecomapper_msgs/WQM.h"
#include "sensor_msgs/NavSatFix.h"
#include "gps_common/GPSFix.h"

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

                /**
                *   Callbacks for ROS
                */
                void dvlCallback(const ecomapper_msgs::DVL::ConstPtr& msg);
                void wqmCallback(const ecomapper_msgs::WQM::ConstPtr& msg);
                void ctd_rhCallback(const ecomapper_msgs::CTD::ConstPtr& msg);
                void fixCallback(const gps_common::GPSFix::ConstPtr& msg);
                void navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

                //Public timelines that connect to ROS
                static TREX::utils::Symbol const dvlObj;
        		static TREX::utils::Symbol const ctd_rhObj;
       			static TREX::utils::Symbol const fixObj;
       			static TREX::utils::Symbol const navSatFixObj;
       			static TREX::utils::Symbol const wqmObj;

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

                ros::NodeHandle*                    m_ros;
                std::list<ros::Subscriber>         m_sub;

                std::list<TREX::transaction::Observation> obs;

                std::mutex Messagelock;

                mutable TREX::utils::SharedVar<bool> m_active;
                boost::asio::deadline_timer m_freq;

                TREX::transaction::TICK m_nextTick;

                std::list<TREX::transaction::goal_id> m_pending;

        };

    }


}

#endif //H_ROS_LISTENER

