/** @file "Platform.cc"
 * @brief Platform plugin implementation
 *
 * @author Jose Pinto <zepinto@gmail.com>
 * @ingroup lsts
 */
#include <iostream>

#include "EuropaExtensions.hh"

#include <trex/utils/Plugin.hh>
#include <trex/utils/LogManager.hh>
#include <trex/domain/IntegerDomain.hh>
#include <trex/domain/FloatDomain.hh>
#include <trex/domain/StringDomain.hh>
#include <trex/domain/BooleanDomain.hh>
#include <trex/domain/EnumDomain.hh>

#include <DUNE/Math/Angles.hpp>
#include <DUNE/Coordinates/WGS84.hpp>
#include "Platform.hh"

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::LSTS;

namespace
{

  /** @brief TREX log entry point */
  SingletonUse<LogManager> s_log;

  /** @brief Platform reactor declaration */
  TeleoReactor::xml_factory::declare<Platform> decl("Platform");

}

namespace TREX
{

  /** @brief Plug-in initialization
   *
   * This function is called by TREX after loading the plug-in.
   * It manage the initialization of this plug-in
   *
   * @ingroup lsts
   */
  void
  initPlugin()
  {
    ::s_log->syslog("plugin.platform", log::info) << "Platform loaded.";
  }

} // TREX

bool estate_posted = false, oplimits_posted = false, was_idle = false;
int last_vstate = -1;
int remote_id = 0;
ControlInterface * Platform::controlInterfaceInstance = 0;

Platform::Platform(TeleoReactor::xml_arg_type arg) :
        TeleoReactor(arg, false), /*m_active_proxy(NULL),*/ m_firstTick(
                true), m_blocked(false)
{

  m_env->setPlatformReactor(this);

  duneport = parse_attr<int>(6002, TeleoReactor::xml_factory::node(arg),
                             "duneport");

  duneip = parse_attr<std::string>("127.0.0.1",
                                   TeleoReactor::xml_factory::node(arg),
                                   "duneip");

  debug = parse_attr<bool>(false, TeleoReactor::xml_factory::node(arg),
                           "debug");

  syslog(log::info) << "Connecting to dune on " << duneip << ":" << duneport;

  // start listening for dune in a new thread
  localport = parse_attr<int>(false, TeleoReactor::xml_factory::node(arg),
                              "localport");

  bfr = new uint8_t[65535];
  receive.bind(localport, Address::Any, true);
  receive.addToPoll(iom);

  syslog(log::info) << "listening on port " << localport << "...";

  provide("estate", false); 		  // declare the state command timeline
  provide("gps", false);
  provide("vstate", false); 		  // declare the vstate command timeline
  provide("oplimits", false); 	  // declare the oplimits timeline
  provide("frefstate", false);    // declare the frefstate timeline
  provide("reference");           // declare the reference timeline
}

bool
Platform::uniqueObservation(TREX::transaction::Observation obs)
{
  std::string timeline = obs.object().str();
  obs_map::iterator it = postedObservations.find(timeline);

  if (it == postedObservations.end() || !it->second->consistentWith(obs))
  {
    postedObservations[timeline].reset(new Observation(obs));

    postObservation(obs, true);
    return true;
  }
  else
  {
    if (debug)
      syslog("debug") << "Found repeated observations:\n" << obs << "\n"
      << *it->second;
  }
  return false;
}

void
Platform::handleTickStart()
{
  // if vehicle is idle and has a new maneuver...
  // post it
  if (m_blocked)
    return;

}

Platform::~Platform()
{
  m_env->setPlatformReactor(NULL);
  if (NULL != bfr)
    delete[] bfr;
  // some clean up may not be very safe though
  //  if (NULL != m_active_proxy)
  //    delete m_active_proxy;
}
void
Platform::setControlInterface(ControlInterface * itf)
{
  controlInterfaceInstance = itf;
}

bool
Platform::synchronize()
{
  try
  {
    Address addr;
    int msg_count = 0;

    received.clear();

    while (iom.poll(0))
    {
      msg_count++;
      uint16_t rv = receive.read((char*)bfr, 65535, &addr);
      IMC::Message * msg = IMC::Packet::deserialize(bfr, rv);
      if (remote_id == 0)
        remote_id = msg->getSource();

      if (received.count(msg->getId()))
        delete received[msg->getId()];
      received[msg->getId()] = msg;
    }

    processState();

    // send an heartbeat to Dune
    IMC::Heartbeat hb;
    sendMsg(hb);

    if (msg_count < 1)
      syslog(log::warn) << "processed " << msg_count << " messages\n";
    else
      syslog(log::info) << "processed " << msg_count << " messages\n";
  }
  catch (std::runtime_error& e)
  {
    syslog(log::error) << "Error during message processing: " << e.what();
    std::cerr << e.what();
    return false;
  }

  return true;
}

void
Platform::handleRequest(goal_id const &g)
{

  Goal * goal = g.get();

  std::string gname = (goal->object()).str();
  std::string gpred = (goal->predicate()).str();
  std::string man_name;
  syslog(log::info) << "handleRequest(" << gname << "." << gpred << ")\n";
}

void
Platform::handleRecall(goal_id const &g)
{
  syslog(log::warn) << "handleRecall(" << g.get()->object().str() << ")";
}

void
Platform::processState()
{

  syslog(log::info) << "processState()\n";
  /* ESTIMATED_STATE */
  if (received.count(IMC::EstimatedState::getIdStatic()))
  {
    IMC::EstimatedState * msg =
        dynamic_cast<IMC::EstimatedState *>(aggregate[IMC::EstimatedState::getIdStatic()]);

    Observation obs("estate", "Position");

    double latitude, longitude;
    latitude = msg->lat;
    longitude = msg->lon;
    WGS84::displace(msg->x, msg->y, &latitude, &longitude);

    obs.restrictAttribute("latitude", FloatDomain(latitude));
    obs.restrictAttribute("longitude", FloatDomain(longitude));
    obs.restrictAttribute("depth", FloatDomain(msg->depth));
    obs.restrictAttribute("altitude", FloatDomain(msg->alt));
    obs.restrictAttribute("height", FloatDomain(msg->alt));

    if (aggregate.count(IMC::NavigationUncertainty::getIdStatic()))
    {
      IMC::NavigationUncertainty * navUnc =
          dynamic_cast<IMC::NavigationUncertainty *>(aggregate[IMC::NavigationUncertainty::getIdStatic()]);
      obs.restrictAttribute("uncertainty",
                            FloatDomain(std::max(navUnc->x, navUnc->y)));
    }
    estate_posted = true;
    postObservation(obs);
  }
  else if (!estate_posted)
  {
    postObservation(Observation("estate", "undefined"));
    estate_posted = true;
  }

  if (received.count(IMC::GpsFix::getIdStatic()))
  {
    IMC::GpsFix * fix =
        dynamic_cast<IMC::GpsFix *>(received[IMC::GpsFix::getIdStatic()]);

    if ((fix->validity & IMC::GpsFix::GFV_VALID_POS) != 0)
      uniqueObservation(Observation("gps", "Valid"));
    else
      uniqueObservation(Observation("gps", "Invalid"));

  }
}

bool
Platform::sendMsg(Message& msg, Address &dest)
{
  DUNE::Utils::ByteBuffer bb;
  try
  {
    msg.setTimeStamp();
    IMC::Packet::serialize(&msg, bb);
    send.write((const char*)bb.getBuffer(), msg.getSerializationSize(), dest,
               duneport);
  }
  catch (std::runtime_error& e)
  {
    syslog("ERROR", log::error) << e.what();
    return false;
  }
  return true;
}

bool
Platform::sendMsg(Message& msg, std::string ip, int port)
{
  DUNE::Utils::ByteBuffer bb;
  try
  {
    msg.setTimeStamp();
    msg.setSource(65000);
    msg.setDestination(remote_id);
    IMC::Packet::serialize(&msg, bb);

    if (debug)
      msg.toText(syslog("debug") << "sending message:\n");

    m_mutex.lock();
    send.write((const char*)bb.getBuffer(), msg.getSerializationSize(),
               Address(ip.c_str()), port);
    m_mutex.unlock();
  }
  catch (std::runtime_error& e)
  {
    syslog("ERROR", log::error) << e.what();
    return false;
  }
  return true;
}

bool
Platform::sendMsg(Message& msg)
{
  return sendMsg(msg, duneip, duneport);
}

bool
Platform::reportToDune(int type, const std::string &message)
{
  return reportToDune(IMC::LogBookEntry::LBET_INFO, "Autonomy.TREX", message);
}

bool
Platform::reportToDune(const std::string &message)
{
  return reportToDune(IMC::LogBookEntry::LBET_INFO, message);
}

bool
Platform::reportToDune(int type, const std::string &context,
    const std::string &text)
{
  IMC::LogBookEntry entry;
  entry.text = text;
  entry.context = context;
  entry.htime = Time::Clock::getSinceEpoch();
  entry.type = type;
  return sendMsg(entry);
}

bool
Platform::reportErrorToDune(const std::string &message)
{
  return reportToDune(IMC::LogBookEntry::LBET_ERROR, message);
}

/*
 * class TREX::LSTS::Platform::log_proxy
 */

// structors 
//Platform::log_proxy::~log_proxy()
//{
//  if (this == m_platform->m_active_proxy)
//  {
//    m_platform->m_active_proxy = NULL;
//  }
//}

// callback

//void Platform::log_proxy::operator()(log::entry::pointer msg) {
//  m_platform->m_active_proxy = this;
//  // Example that display error/warnings on std::cerr
//
//  std::ostringstream ss;
//  if( msg->is_dated() )
//    ss<<'@'<<msg->date()<<' ';
//
//  ss<<msg->content();
//
//  if( log::warn==msg->kind() )
//    m_platform->reportToDune(IMC::LogBookEntry::LBET_WARNING, who.str(), ss.str());
//  else if ( log::error==msg->kind() )
//    m_platform->reportToDune(IMC::LogBookEntry::LBET_ERROR, who.str(), ss.str());
//  else if (TeleoReactor::obs==msg->kind() )
//    m_platform->reportToDune(IMC::LogBookEntry::LBET_INFO, who.str(), ss.str());
//}

