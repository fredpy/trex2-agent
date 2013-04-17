#include "turtlesimpub.hh"
#include <trex/domain/FloatDomain.hh>

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::TREXturtlesim;

Symbol const TurtleSimPub::PoseObj("pose");
Symbol const TurtleSimPub::ControlObj("turtle");

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
    m_sub.push_back(m_ros->subscribe("turtle1/pose", 10, &TurtleSimPub::poseCallback, this));

    m_pub.insert(make_pair(ControlObj, m_ros->advertise<turtlesim::Velocity>("turtle1/command_velocity", 1)));
    spinner->start();

    m_nextTick = getCurrentTick();
    //Post the inital state of the turtle
    stop();
}

void TurtleSimPub::poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    TICK cur = getCurrentTick();
    if(m_nextPoseTick<=cur)
    {
        m_nextPoseTick = cur+1;
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
}


bool TurtleSimPub::synchronize()
{
    bool postedToTurtle = false;
    TICK cur = getCurrentTick();
    if(m_nextTick<=cur)
    {
        while(!m_pending.empty())
        {
            if(m_pending.front()->startsAfter(cur))
            {
                if(m_pending.front()->startsBefore(cur)+1)
                {
                    if(m_pending.front()->predicate()==Symbol("stop"))
                    {
                        stop();
                    } else {
                        move(m_pending.front());
                    }
                    m_nextTick = cur+m_pending.front()->getDuration().lowerBound().value();
                    m_pending.pop_front();
                    postedToTurtle = true;
                }
                break;
            } else {
                m_pending.pop_front();
            }
        }
    }
    if(!postedToTurtle)
    {
        updateTurtle();
    }
    return true;
}

void TurtleSimPub::move(TREX::transaction::goal_id const &g)
{
    Observation state(ControlObj, "move");
    turtlesim::Velocity msg;
    msg.linear = boost::any_cast<double>(g->getAttribute(Symbol("lin_vel")).domain().getSingleton());
    linear_velocity = msg.linear;
    state.restrictAttribute("lin_vel", FloatDomain(msg.linear));
    msg.angular = boost::any_cast<double>(g->getAttribute(Symbol("ang_vel")).domain().getSingleton());
    angular_velocity = msg.angular;
    state.restrictAttribute("ang_vel", FloatDomain(msg.angular));
    m_pub[ControlObj].publish(msg);
    postObservation(state);
}

void TurtleSimPub::stop()
{
    Observation state(ControlObj, Symbol("stop"));
    angular_velocity = 0;
    linear_velocity = 0;
    postObservation(state);
}

void TurtleSimPub::updateTurtle()
{
    turtlesim::Velocity msg;
    msg.linear = linear_velocity;
    msg.angular = angular_velocity;
    m_pub[ControlObj].publish(msg);
}

void TurtleSimPub::handleRequest(goal_id const &g)
{
    if(g->predicate()==Symbol("move") || g->predicate()==Symbol("stop"))
    {
        IntegerDomain::bound lo = g->getStart().lowerBound();
        if(lo.isInfinity())
        {
            m_pending.push_front(g);
        } else {
            std::list<goal_id>::iterator i = m_pending.begin();
            TICK val = lo.value();
            for(; i!=m_pending.end(); ++i)
            {
                if((*i)->startsAfter(val))
                    break;
            }
            m_pending.insert(i,g);
        }
    }
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
