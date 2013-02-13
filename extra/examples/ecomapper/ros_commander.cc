#include "ros_commander.hh"
#include <string>

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::ecomapper;

Symbol const Ros_Commander::ros_commanderObj("ros_commander");

Ros_Commander::Ros_Commander(TeleoReactor::xml_arg_type arg)
    :TeleoReactor(arg,false)
{
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "trex2_ros_commander" , ros::init_options::AnonymousName);
    //m_ros = new ros::NodeHandle();
    client = new actionlib::SimpleActionClient<ecomapper_msgs::EcomapperCommandAction>
                                ("trex2_client_commander");

    syslog()<<"I want to own "<<ros_commanderObj;
    provide(ros_commanderObj);

}

Ros_Commander::~Ros_Commander()
{
    delete client;
}


void Ros_Commander::handleInit()
{
    sonar.enable = 0;
    sonar.frequency = ecomapper_msgs::SonarSettings::FREQUENCY_HIGH;
    sonar.range = 30;
    sonar.channel = ecomapper_msgs::SonarSettings::CHANNEL_BOTH;
    sonar.type = ecomapper_msgs::SonarSettings::SONAR_TYPE_IMAGENEX;
    sonar.gain = 8;
}

bool Ros_Commander::sendCommand(double const& longitude, double const& latitude)
{
    ecomapper_msgs::EcomapperCommandGoal ecoGoal = ecomapper_msgs::EcomapperCommandGoal();
    ecoGoal.command_type = ecomapper_msgs::EcomapperCommandGoal::COMMAND_TYPE_WAYPOINTS;
    ecomapper_msgs::Waypoint wp = ecomapper_msgs::Waypoint();
    wp.longitude = longitude;
    wp.latitude = latitude;
    wp.speed = 1.5;
    wp.depth = 0;
    wp.undulate_depth =0;
    wp.sonar_settings = sonar;
    wp.max_pitch = 20;
    ecoGoal.waypoints.push_back(wp);
    client->sendGoal(ecoGoal, boost::bind(&Ros_Commander::goalCompleted, this, _1, _2), NULL, NULL);
}

void Ros_Commander::goalCompleted(const actionlib::SimpleClientGoalState& state,
                    const ecomapper_msgs::EcomapperCommandResultConstPtr& result)
{

}

bool Ros_Commander::synchronize()
{
    TICK cur = getCurrentTick();
    if(m_nextTick<=cur)
    {
        while(!m_pending.empty())
        {
            if(m_pending.front()->startsAfter(cur))
            {
                if(m_pending.front()->startsBefore(cur))
                {
                    m_nextTick = cur+m_pending.front()->getDuration().lowerBound().value();
                    m_pending.pop_front();
                }
                break;
            } else {
                m_pending.pop_front();
            }
        }
    }
    return true;
}

void Ros_Commander::handleRequest(goal_id const &g)
{

}

void Ros_Commander::handleRecall(goal_id const &g)
{
    std::list<goal_id>::iterator i = m_pending.begin();
    for(; i!=m_pending.end(); ++i)
    {
        if(*i==g)
        {
            m_pending.erase(i);
            return;
        }
    }
}
