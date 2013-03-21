#include "turtlesimpub.hh"
#include <trex/domain/FloatDomain.hh>

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::TREXturtlesim;

Symbol const TurtleSimPub::PoseObj("/turtle1/pose");
Symbol const TurtleSimPub::ControlObj("/turtle1/command_velocity");

TurtleSimPub::TurtleSimPub(TeleoReactor::xml_arg_type arg)
    :TeleoReactor(arg,false)
{
    int argc = 0; char **argv = NULL; //Variables for the ros::init
    ros::init(argc, argv, "trex2_TurtleSimPub" , ros::init_options::AnonymousName);
    m_ros = new ros::NodeHandle();
    spinner = new ros::AsyncSpinner(0);

    syslog()<<"I want to own "<<PoseObj; // Message for the log manager
    provide(PoseObj); //Creates the timeline in TREX
    syslog()<<"I want to own "<<ControlObj; // Message for the log manager
    provide(ControlObj); //Creates the timeline in TREX

}

TurtleSimPub::~TurtleSimPub()
{
    delete m_ros;
    spinner->stop();
    delete spinner;
}


void TurtleSimPub::handleInit()
{
    m_sub.push_back(m_ros->subscribe(PoseObj.str(), 10, &TurtleSimPub::poseCallback, this));

    m_pub.insert(make_pair(ControlObj, m_ros->advertise<turtlesim::Velocity>("turtle1/command_velocity", 1)));
    spinner->start();
}

void TurtleSimPub::poseCallback(const turtlesim::PoseConstPtr& msg)
{
    Messagelock.lock();
    Observation state(PoseObj, Symbol(PoseObj.str())); //Creates observation on Pose timeline
    const float& x = msg->x; //Get message variable x and copy into float
    state.restrictAttribute("x", FloatDomain(x)); // Restrict the "x" attribute in the observation
    const float& y = msg->y;
    state.restrictAttribute("y", FloatDomain(y));
    const float& theta = msg->theta;
    state.restrictAttribute("theta", FloatDomain(theta));
    const float& linear_velocity = msg->linear_velocity;
    state.restrictAttribute("linear_velocity", FloatDomain(linear_velocity));
    const float& angular_velocity = msg->angular_velocity ;
    state.restrictAttribute("angular_velocity", FloatDomain(angular_velocity));
    postObservation(state);
    Messagelock.unlock();
}


bool TurtleSimPub::synchronize()
{
    TICK cur = getCurrentTick();
    if(m_nextTick<=cur)
    {
        turtlesim::Velocity msg;
        msg.linear = 1;
        msg.angular = .5;
        m_pub[ControlObj].publish(msg);
        if(!obs.empty())
        {
            //postObservation(obs.front());
            //obs.clear();
        }
        while(!m_pending.empty())
        {
            if(m_pending.front()->startsAfter(cur))
            {
                if(m_pending.front()->startsBefore(cur))
                {
                    //setValue(m_pending.front());
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

void TurtleSimPub::handleRequest(goal_id const &g)
{

}

void TurtleSimPub::handleRecall(goal_id const &g)
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
