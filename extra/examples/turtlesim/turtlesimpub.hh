#ifndef H_TURTLESIMPUB
#define H_TURTLESIMPUB

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Velocity.h>
#include <turtlesim/Color.h>

#include <list>
#include <map>
#include <mutex>

#include <trex/transaction/TeleoReactor.hh>

namespace TREX {

    namespace TREXturtlesim {

        class TurtleSimPub :public TREX::transaction::TeleoReactor
        {
            public:
                TurtleSimPub(TREX::transaction::TeleoReactor::xml_arg_type arg);
                ~TurtleSimPub();

                void poseCallback(const turtlesim::PoseConstPtr& msg);

                static TREX::utils::Symbol const PoseObj;
                static TREX::utils::Symbol const ControlObj;

            private:
                void handleInit();
                bool synchronize();
                void handleRequest(TREX::transaction::goal_id const &g);
                void handleRecall(TREX::transaction::goal_id const &g);


                TREX::utils::SingletonUse<TREX::utils::LogManager> m_log;

                ros::NodeHandle*                    m_ros;
                std::list<ros::Subscriber>          m_sub;
                std::map<TREX::utils::Symbol, ros::Publisher> m_pub;
                ros::AsyncSpinner* spinner;

                std::mutex Messagelock;

                std::list<TREX::transaction::Observation> obs;

                TREX::transaction::TICK m_nextTick;
                TREX::transaction::TICK m_nextPoseTick;

                std::list<TREX::transaction::goal_id> m_pending;

        };

    }


}

#endif //H_TURTLESIMPUB

