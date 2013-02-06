#include "ros_listener.hh"
#include <trex/domain/StringDomain.hh>
#include <string>

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::ecomapper;

Symbol const Ros_Listener::latitudeObj("Latitude");

Ros_Listener::Ros_Listener(TeleoReactor::xml_arg_type arg)
    :TeleoReactor(arg,false), m_active(false), m_freq(m_log->service())
{
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "trex2" , ros::init_options::AnonymousName);
    m_ros = new ros::NodeHandle();

    syslog()<<"I want to own "<<latitudeObj;
    provide(latitudeObj);
}

Ros_Listener::~Ros_Listener()
{
    delete m_ros;
}


void Ros_Listener::latitudeCallback(const std_msgs::String::ConstPtr& msg)
{
    //std::cout<<"Message :"<<(*msg)<<std::endl;
    Messagelock.lock();
    Observation latitude_state(latitudeObj, Symbol("location"));
    std::string message = msg->data;
    latitude_state.restrictAttribute("data", StringDomain(message));
    obs.push_back(latitude_state);
    Messagelock.unlock();
}

void Ros_Listener::handleInit()
{
    m_sub.push_back(m_ros->subscribe("chatter", 10, &Ros_Listener::latitudeCallback, this));
    start();
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
