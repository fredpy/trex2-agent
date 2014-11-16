#include "ros_commander.hh"
#include <string>
#include <boost/thread.hpp>

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::ecomapper;

Symbol const Ros_Commander::ros_commanderObj("ros_commander");

Ros_Commander::Ros_Commander(TeleoReactor::xml_arg_type arg)
    :TeleoReactor(arg,false), goalLoaded(false), spinner(0)
{
    syslog()<<"I want to own "<<ros_commanderObj;
    provide(ros_commanderObj);

    //int argc = 0;
    //char **argv = NULL;
    //ros::init(argc, argv, "trex2_ros_commander" , ros::init_options::AnonymousName);

    node = new ros::NodeHandle;

    client = new actionlib::SimpleActionClient<ecomapper_msgs::EcomapperCommandAction>
                                ("trex2_client_commander");
    spinner.start();

    bool clientConnected = client->waitForServer(ros::Duration(5.0));
    if(clientConnected)
        syslog()<<"Ros_Commander connected to client server";
    else
        syslog("Warning")<<"Ros_Commander did not connect to client server";

}

Ros_Commander::~Ros_Commander()
{
    delete client;
    delete node;
    spinner.stop();
}


void Ros_Commander::handleInit()
{
    sonar.enable = 0;
    sonar.frequency = ecomapper_msgs::SonarSettings::FREQUENCY_HIGH;
    sonar.range = 30;
    sonar.channel = ecomapper_msgs::SonarSettings::CHANNEL_BOTH;
    sonar.type = ecomapper_msgs::SonarSettings::SONAR_TYPE_IMAGENEX;
    sonar.gain = 8;

    ///First observation
    postObservation(std::string("open"));

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
    if(client->isServerConnected())
    {
        client->sendGoal(ecoGoal, boost::bind(&Ros_Commander::goalCompleted, this, _1, _2), NULL, NULL);
        postObservation(std::string("closed"));
        goalLoaded = true;
    } else
        syslog("Warning")<<"Ros_Commander did not send goal because it is not connected to client";
}

void Ros_Commander::goalCompleted(const actionlib::SimpleClientGoalState& state,
                    const ecomapper_msgs::EcomapperCommandResultConstPtr& result)
{
    postObservation(std::string("open"));
    goalLoaded = false;
}

void Ros_Commander::postObservation(std::string pred)
{
    Observation obs = Observation(ros_commanderObj, pred);
    TREX::transaction::TeleoReactor::postObservation(obs);
}

bool Ros_Commander::synchronize()
{
    TICK cur = getCurrentTick();
    if(m_nextTick<=cur)
    {
        if(client->isServerConnected() && goalLoaded)
        {
            actionlib::SimpleClientGoalState state = client->getState();
            postObservation(state.toString());
        } else if(!client->isServerConnected())
            syslog("Warning")<<"Ros_Commander is not connected to client";
    }
    return true;
}

void Ros_Commander::handleRequest(goal_id const &g)
{
    double latitude, longitude;
    try
    {
        latitude = boost::any_cast<double>(g->getAttribute(Symbol("latitude")).domain().getSingleton());
        longitude = boost::any_cast<double>(g->getAttribute(Symbol("longitude")).domain().getSingleton());
    } catch (boost::bad_any_cast& err) {
        syslog("Error")<<"Ros_commander casting error in handleRequest";
        return;
    }
    sendCommand(longitude, latitude);
}

void Ros_Commander::handleRecall(goal_id const &g)
{
    ///Can't cancel a specifc goal but the current running goal
    client->cancelGoal();
}
