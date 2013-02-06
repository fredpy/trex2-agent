#include "ros_listener.hh"
#include <trex/domain/StringDomain.hh>
#include <string>

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::ecomapper;

Symbol const Ros_Listener::dvlObj("dvl");
Symbol const Ros_Listener::ctd_rhObj("ctd_rh");
Symbol const Ros_Listener::fixObj("fix");

Ros_Listener::Ros_Listener(TeleoReactor::xml_arg_type arg)
    :TeleoReactor(arg,false), m_active(false), m_freq(m_log->service())
{
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "trex2" , ros::init_options::AnonymousName);
    m_ros = new ros::NodeHandle();

    syslog()<<"I want to own "<<dvlObj;
    provide(dvlObj);
    syslog()<<"I want to own "<<ctd_rhObj;
    provide(ctd_rhObj);
    syslog()<<"I want to own "<<fixObj;
    provide(fixObj);
}

Ros_Listener::~Ros_Listener()
{
    delete m_ros;
}


void Ros_Listener::handleInit()
{
    m_sub.push_back(m_ros->subscribe(dvlObj.str(), 10, &Ros_Listener::dvlCallback, this));
    m_sub.push_back(m_ros->subscribe(ctd_rhObj.str(), 10, &Ros_Listener::ctd_rhCallback, this));
    m_sub.push_back(m_ros->subscribe(fixObj.str(), 10, &Ros_Listener::fixCallback, this));
    start();
}

void Ros_Listener::dvlCallback(const std_msgs::String::ConstPtr& msg)
{
    Messagelock.lock();
    Observation dvl_state(dvlObj, Symbol("location"));
    std::string message = msg->data;
    dvl_state.restrictAttribute("data", StringDomain(message));
    obs.push_back(dvl_state);
    Messagelock.unlock();
}

void Ros_Listener::ctd_rhCallback(const std_msgs::String::ConstPtr& msg)
{

}

void Ros_Listener::fixCallback(const std_msgs::String::ConstPtr& msg)
{

}

bool Ros_Listener::synchronize()
{
    TICK cur = getCurrentTick();
    if(m_nextTick<=cur)
    {
        if(!obs.empty())
        {
            postObservation(obs.front());
            obs.clear();
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

void Ros_Listener::handleRequest(goal_id const &g)
{

}

void Ros_Listener::handleRecall(goal_id const &g)
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



/**
*   Below controls the ROS node that is used to link to
*   ROS. It controls the spin rate of the messages.
*/

bool Ros_Listener::started() const {
  TREX::utils::SharedVar<bool>::scoped_lock l(m_active);
  return *m_active;
}

void Ros_Listener::start() {
  bool should_start = false;
  {
    TREX::utils::SharedVar<bool>::scoped_lock l(m_active);
    if( !*m_active && ::ros::ok() ) {
      should_start = true;
      *m_active = true;
    }
  }
  if( should_start )
    m_log->service().post(boost::bind(&Ros_Listener::spin_cb, this));
}

void Ros_Listener::spin_cb() {
  if( started() ) {
    if( ::ros::ok() ) {
      // Set my timer
      m_freq.expires_from_now(boost::posix_time::milliseconds(100)); // 10Hz is more than enough
      // manage things from ros
      ::ros::spinOnce();

      m_freq.async_wait(boost::bind(&Ros_Listener::spin_cb, this));
    } else
      stop();
  }
}
