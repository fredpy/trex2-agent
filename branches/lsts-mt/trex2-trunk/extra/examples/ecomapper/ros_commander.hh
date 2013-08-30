#ifndef H_ROS_COMMANDER
#define H_ROS_COMMANDER

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "ecomapper_msgs/EcomapperCommandAction.h"
#include "ecomapper_msgs/EcomapperCommandGoal.h"
#include "ecomapper_msgs/SonarSettings.h"
#include "ecomapper_msgs/Waypoint.h"
#include "std_msgs/String.h"

#include <trex/transaction/TeleoReactor.hh>
#include <iostream>
#include <list>

namespace TREX {

    namespace ecomapper {

        class Ros_Commander :public TREX::transaction::TeleoReactor
        {
            typedef actionlib::SimpleActionClient<ecomapper_msgs::EcomapperCommandAction> RosClient;

            public:
                Ros_Commander(TREX::transaction::TeleoReactor::xml_arg_type arg);
                ~Ros_Commander();

                static TREX::utils::Symbol const ros_commanderObj;

            private:
                void handleInit();
                bool synchronize();
                void handleRequest(TREX::transaction::goal_id const &g);
                void handleRecall(TREX::transaction::goal_id const &g);
                void postObservation(std::string pred);

                /** ROS handle */
                ros::NodeHandle* node;

                /** ROS Client */
                RosClient* client;
                /** Sends the waypoint commands to ROS */
                bool sendCommand(double const& longitude, double const& latitude);
                /** Callback for when goals are finished */
                void goalCompleted(const actionlib::SimpleClientGoalState& state,
                                    const ecomapper_msgs::EcomapperCommandResultConstPtr& result);
                /** Sonar settings for goals defined in init function */
                ecomapper_msgs::SonarSettings sonar;


                TREX::utils::SingletonUse<TREX::utils::LogManager> m_log;

                //std::mutex Messagelock;
                ros::AsyncSpinner spinner;
                /** True if a goal is being executed by ROS */
                bool goalLoaded;

                TREX::transaction::TICK m_nextTick;

                std::list<TREX::transaction::goal_id> m_pending;

        };

    }


}

#endif //H_ROS_COMMANDER

