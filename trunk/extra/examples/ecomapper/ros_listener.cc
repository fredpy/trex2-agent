#include "ros_listener.hh"
#include <trex/domain/StringDomain.hh>
#include <trex/domain/FloatDomain.hh>
#include <string>

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::ecomapper;

Symbol const Ros_Listener::stateObj("state");
Symbol const Ros_Listener::dvlObj("dvl");
Symbol const Ros_Listener::ctd_rhObj("ctd_rh");
Symbol const Ros_Listener::fixObj("extended_fix");
Symbol const Ros_Listener::navSatFixObj("fix");
Symbol const Ros_Listener::wqmObj("wqm");

Ros_Listener::Ros_Listener(TeleoReactor::xml_arg_type arg)
    :TeleoReactor(arg,false), m_active(false), m_freq(m_log->service())
{
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "trex2_ros_listener" , ros::init_options::AnonymousName);
    m_ros = new ros::NodeHandle();

    syslog()<<"I want to own "<<dvlObj;
    provide(stateObj);
    syslog()<<"I want to own "<<dvlObj;
    provide(dvlObj);
    syslog()<<"I want to own "<<ctd_rhObj;
    provide(ctd_rhObj);
    syslog()<<"I want to own "<<fixObj;
    provide(fixObj);
    syslog()<<"I want to own "<<navSatFixObj;
    provide(navSatFixObj);
    syslog()<<"I want to own "<<wqmObj;
    provide(wqmObj);
}

Ros_Listener::~Ros_Listener()
{
    delete m_ros;
}


void Ros_Listener::handleInit()
{
    m_sub.push_back(m_ros->subscribe(stateObj.str(), 10, &Ros_Listener::stateCallback, this));
    m_sub.push_back(m_ros->subscribe(dvlObj.str(), 10, &Ros_Listener::dvlCallback, this));
    m_sub.push_back(m_ros->subscribe(ctd_rhObj.str(), 10, &Ros_Listener::ctd_rhCallback, this));
    m_sub.push_back(m_ros->subscribe(fixObj.str(), 10, &Ros_Listener::fixCallback, this));
    m_sub.push_back(m_ros->subscribe(navSatFixObj.str(), 10, &Ros_Listener::navSatFixCallback, this));
    m_sub.push_back(m_ros->subscribe(wqmObj.str(), 10, &Ros_Listener::wqmCallback, this));
    start();
}

void Ros_Listener::stateCallback(const ecomapper_msgs::State::ConstPtr& msg)
{
    Messagelock.lock();
    Observation state(stateObj, Symbol(stateObj.str()));
    const double& latitude = msg->location.latitude;
    state.restrictAttribute("latitude", FloatDomain(latitude));
    const double& longitude = msg->location.longitude;
    state.restrictAttribute("longitude", FloatDomain(longitude));
    const double& depth = msg->location.depth;
    state.restrictAttribute("depth", FloatDomain(depth));
    const double& altitude = msg->location.altitude;
    state.restrictAttribute("altitude", FloatDomain(altitude));
    const double& wp_number = msg->wp_number;
    state.restrictAttribute("wp_number", FloatDomain(wp_number));
    const double& wp_distance = msg->wp_distance;
    state.restrictAttribute("wp_distance", FloatDomain(wp_distance));
    postObservation(state);
    Messagelock.unlock();
}



void Ros_Listener::dvlCallback(const ecomapper_msgs::DVL::ConstPtr& msg)
{
    Messagelock.lock();
    Observation dvl_state(dvlObj, Symbol(dvlObj.str()));
    const double& depth = msg->depth;
    dvl_state.restrictAttribute("depth", FloatDomain(depth));
    const double& altitude = msg->altitude;
    dvl_state.restrictAttribute("altitude", FloatDomain(altitude));
    const double& vel_x = msg->vel_x;
    dvl_state.restrictAttribute("vel_x", FloatDomain(vel_x));
    const double& vel_y = msg->vel_y;
    dvl_state.restrictAttribute("vel_y", FloatDomain(vel_y));
    const double& vel_z = msg->vel_z;
    dvl_state.restrictAttribute("vel_z", FloatDomain(vel_z));
    const double& dist_x = msg->dist_x;
    dvl_state.restrictAttribute("dist_x", FloatDomain(dist_x));
    const double& dist_y = msg->dist_y;
    dvl_state.restrictAttribute("dist_y", FloatDomain(dist_y));
    const double& heading = msg->heading;
    dvl_state.restrictAttribute("heading", FloatDomain(heading));
    postObservation(dvl_state);
    //obs.push_back(dvl_state);
    Messagelock.unlock();
}

void Ros_Listener::wqmCallback(const ecomapper_msgs::WQM::ConstPtr& msg)
{
    Messagelock.lock();
    Observation state(wqmObj, Symbol(wqmObj.str()));
    const double& depth = msg->depth;
    state.restrictAttribute("depth", FloatDomain(depth));
    const double& conductivity = msg->conductivity;
    state.restrictAttribute("conductivity", FloatDomain(conductivity));
    const double& temperature = msg->temperature;
    state.restrictAttribute("temperature", FloatDomain(temperature));
    const double& salinity = msg->salinity;
    state.restrictAttribute("salinity", FloatDomain(salinity));
    const double& pressure = msg->pressure;
    state.restrictAttribute("pressure", FloatDomain(pressure));

    postObservation(state);
    //obs.push_back(state);
    Messagelock.unlock();
}

void Ros_Listener::ctd_rhCallback(const ecomapper_msgs::CTD::ConstPtr& msg)
{
	Messagelock.lock();
    Observation ctd_rh_state(ctd_rhObj, Symbol(ctd_rhObj.str()));
    const double& conductivity = msg->conductivity;
    ctd_rh_state.restrictAttribute("conductivity", FloatDomain(conductivity));
    const double& temperature = msg->temperature;
    ctd_rh_state.restrictAttribute("temperature", FloatDomain(temperature));
    const double& salinity = msg->salinity;
    ctd_rh_state.restrictAttribute("salinity", FloatDomain(salinity));
    const double& sound_speed = msg->sound_speed;
    ctd_rh_state.restrictAttribute("sound_speed", FloatDomain(sound_speed));
    postObservation(ctd_rh_state);
    //obs.push_back(ctd_rh_state);
    Messagelock.unlock();
}

void Ros_Listener::fixCallback(const gps_common::GPSFix::ConstPtr& msg)
{
	Messagelock.lock();
    Observation fix_state(fixObj, Symbol(fixObj.str()));
    const double& latitude = msg->latitude;
    fix_state.restrictAttribute("latitude", FloatDomain(latitude));
    const double& longitude = msg->longitude;
    fix_state.restrictAttribute("longitude", FloatDomain(longitude));
    const double& altitude = msg->altitude;
    fix_state.restrictAttribute("altitude", FloatDomain(altitude));
    const double& speed = msg->speed;
    fix_state.restrictAttribute("speed", FloatDomain(speed));
    postObservation(fix_state);
    //obs.push_back(fix_state);
    Messagelock.unlock();
}

void Ros_Listener::navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	Messagelock.lock();
    Observation state(navSatFixObj, Symbol(navSatFixObj.str()));
    const double& latitude = msg->latitude;
    state.restrictAttribute("latitude", FloatDomain(latitude));
    const double& longitude = msg->longitude;
    state.restrictAttribute("longitude", FloatDomain(longitude));
    const double& altitude = msg->altitude;
    state.restrictAttribute("altitude", FloatDomain(altitude));
    postObservation(state);
    //obs.push_back(state);
    Messagelock.unlock();
}

bool Ros_Listener::synchronize()
{
    TICK cur = getCurrentTick();
    if(m_nextTick<=cur)
    {
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
      m_freq.expires_from_now(boost::posix_time::milliseconds(10)); // 10Hz is more than enough
      // manage things from ros
      ::ros::spinOnce();

      m_freq.async_wait(boost::bind(&Ros_Listener::spin_cb, this));
    } else
      stop();
  }
}
