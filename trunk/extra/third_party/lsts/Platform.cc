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

#include <Dune/Math/Angles.hpp>
#include <Dune/Coordinates/WGS84.hpp>
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

bool estate_posted = false, oplimits_posted = false, was_idle = false, supervision_posted = false;
int last_vstate = -1;
int remote_id = 0;
bool supervision_blocked = false;
std::string current_goal, next_goal;
ControlInterface * Platform::controlInterfaceInstance = 0;


Platform::Platform(TeleoReactor::xml_arg_type arg) :
                                              TeleoReactor(arg, false), m_active_proxy(NULL),
                                              m_on(parse_attr<bool>(false, TeleoReactor::xml_factory::node(arg), "state")), m_firstTick(true),
                                              sent_command(NULL), m_blocked(false), commandToBePosted(NULL), postedCommand(NULL)
{

  manager().on_new_log(log_proxy(*this));
  //syslog(log::warn)<<"This message should appeear in stderr";

  m_env->setPlatformReactor(this);

  duneport = parse_attr<int>(6002, TeleoReactor::xml_factory::node(arg),
                             "duneport");

  duneip = parse_attr<std::string>("127.0.0.1", TeleoReactor::xml_factory::node(arg),
                                   "duneip");

  debug = parse_attr<bool>(false, TeleoReactor::xml_factory::node(arg), "debug");


  m_rpm_speed_factor = parse_attr<double>(-1, TeleoReactor::xml_factory::node(arg),
                                          "rpm_speed_factor");

  if (m_rpm_speed_factor != -1)
    m_use_rpm = true;
  else
    m_use_rpm = false;


  m_loiter_radius = parse_attr<double>(15, TeleoReactor::xml_factory::node(arg),
                                       "loiter_radius");

  m_skeeping_radius = parse_attr<double>(7.5, TeleoReactor::xml_factory::node(arg),
                                         "skeeping_radius");

  syslog(log::info) << "Connecting to dune on " << duneip << ":" 
      << duneport;

  // start listening for dune in a new thread
  localport = parse_attr<int>(false, TeleoReactor::xml_factory::node(arg),
                              "localport");

  bfr = new uint8_t[65535];
  receive.bind(localport, Address::Any, true);
  receive.addToPoll(iom);

  syslog(log::info) << "listening on port " << localport << "...";

  provide("estate", false); 		  // declare the state command timeline
  provide("vstate", false); 		  // declare the vstate command timeline
  provide("command"); 			      // declare the command timeline
  provide("oplimits", false); 	  // declare the oplimits timeline
  provide("maneuver", false);     // declare the maneuver timeline
  provide("supervision", false);  // declare the supervision timeline
  provide("gps", false);

  IMC::LoggingControl startLog;
  startLog.op = IMC::LoggingControl::COP_REQUEST_START;
  startLog.name = "TREX";
  sendMsg(startLog);

  IMC::CacheControl m;
  m.op = IMC::CacheControl::COP_LOAD;
  sendMsg(m);

  sleep(3);
}

bool Platform::uniqueObservation(TREX::transaction::Observation obs)
{
  std::string timeline = obs.object().str();
  obs_map::iterator it = postedObservations.find(timeline);

  //std::cerr << "Observation tested: \"" << timeline << "\" -- " << obs << " at " << getCurrentTick() << std::endl;

  if (it == postedObservations.end() || !it->second->consistentWith(obs)) {
    postedObservations[timeline].reset(new Observation(obs));
    //std::cerr << "Observation posted: \"" << timeline << "\" -- " << obs << " at " << getCurrentTick() << std::endl;

    postObservation(obs, true);
    return true;
  }
  else {
    if (debug)
      syslog("debug") << "Found repeated observations:\n" << obs << "\n" << *it->second;
  }
  return false;
}

void Platform::handleTickStart()
{
  // if vehicle is idle and has a new maneuver...
  // post it
  if (m_blocked)
    return;

  //IMC::VehicleState *curState = dynamic_cast<IMC::VehicleState *>(aggregate[IMC::VehicleState::getIdStatic()]);

  if (commandToBePosted != NULL && tickWhenToPost <= getCurrentTick() /*&& curState->op_mode == IMC::VehicleState::VS_SERVICE*/)
  {

    IMC::VehicleState * msg =
        dynamic_cast<IMC::VehicleState *>(aggregate[IMC::VehicleState::getIdStatic()]);

    if (msg != NULL && msg->op_mode == IMC::VehicleState::VS_SERVICE)
    {
      syslog(log::warn) << "handling at tick " << getCurrentTick();
      if (sendMsg(*commandToBePosted)) {
        reportToDune("Maneuver dispatched");
        syslog(log::warn) << "Dispatched maneuver:";
        commandToBePosted->toText(syslog(warn));

        postedCommand = commandToBePosted;
        //delete commandToBePosted;
        commandToBePosted = NULL;

        current_goal = next_goal;
        next_goal = "";
      }
      else {
        syslog(log::error) << "Unable to send message";
      }
    }
  }
  else if (commandToBePosted != NULL){
    syslog(log::warn) << "Not handling right now... waiting for tick " << tickWhenToPost;
  }
}

Platform::~Platform()
{
  m_env->setPlatformReactor(NULL);
  if( NULL!=bfr )
    delete[] bfr;
  // some clean up may not be very safe though
  if( NULL!=m_active_proxy )
    delete m_active_proxy;
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
    //bool estate_changed = false, vstate_changed = false, oplimits_changed =
    // 		false, gpsfix_changed = false;

    //entityStates.clear();
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

      if (msg->getId() == IMC::VehicleCommand::getIdStatic())
      {
        IMC::VehicleCommand * cmd = dynamic_cast<IMC::VehicleCommand *>(msg);
        if (cmd->type == IMC::VehicleCommand::VC_REQUEST)
        {
          lastCommand = *cmd;
        }
      }



      if (msg->getId() == IMC::TrexCommand::getIdStatic())
      {
        IMC::TrexCommand * command = dynamic_cast<IMC::TrexCommand*>(msg);

        switch (command->command)
        {
          case IMC::TrexCommand::OP_POST_GOAL:
            syslog(log::info) << "received (" << command->goalid << "): " << command->goalxml;
            if (controlInterfaceInstance) {
              controlInterfaceInstance->proccess_message(command->goalxml);
            }
            break;
          case IMC::TrexCommand::OP_ENABLE:
            syslog(log::warn) << "Enable TREX command received";
            // post active observation ...
            uniqueObservation(Observation("supervision", "Active"));
            reportToDune("Activate TREX command received");
            m_blocked = false;

            break;
          case IMC::TrexCommand::OP_DISABLE:
            syslog(log::warn) << "Disable TREX command received";
            // post blocked observation ...
            uniqueObservation(Observation("supervision", "Blocked"));
            reportToDune("Disable TREX command received");
            m_blocked = true;
            break;
        }
      }

      if (msg->getId() == IMC::Abort::getIdStatic())
      {
        syslog(log::warn) << "Abort received";
        // post blocked observation ...
        uniqueObservation(Observation("supervision", "Blocked"));
        reportToDune("Disabling TREX due to abort detection");
      }
    }

    std::map<uint16_t, IMC::Message *>::iterator it;

    for (it = received.begin(); it != received.end(); it++)
      aggregate[it->first] = it->second;

    processState();

    if (!supervision_posted) {
      uniqueObservation(Observation("supervision", "Active"));
      supervision_posted = true;
    }

    // send an heartbeat to Dune
    IMC::Heartbeat hb;
    sendMsg(hb);

    if( msg_count<1 )
      syslog(log::warn) << "processed " << msg_count << " messages\n";
    std::cout << "processed " << msg_count << " messages\n";
  }
  catch (std::runtime_error& e)
  {
    syslog(log::error)<<"Error during message processing: "<<e.what();
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

  std::ostringstream ss;
  ss << "TREX_goal_" << g;
  man_name = ss.str();

  tickWhenToPost = g.get()->getStart().getTypedLower<long long, true>();

  if (gname.compare("command") == 0 && gpred.compare("Idle") == 0) {
    commandToBePosted = idleCommand(man_name);
    return;
  }

  if (gname.compare("command") == 0 && gpred.compare("Maneuver") == 0 && commandToBePosted == NULL)
  {
    boost::optional<long long> timeout;

    if (g->getDuration().hasUpper()) {

      timeout = g->getDuration().getTypedUpper<long long, true>();

      if (*timeout > 2)
        (*timeout) -= 2;
      else
        timeout = 1;
    }

    Variable latitude = goal->getAttribute("latitude");
    Variable longitude = goal->getAttribute("longitude");
    Variable depth = goal->getAttribute("depth");
    Variable secs = goal->getAttribute("secs");
    Variable speed = goal->getAttribute("speed");

    if (!latitude.domain().isSingleton() && !longitude.domain().isSingleton()
        && depth.domain().isSingleton()) {
      double dep_v = depth.domain().getTypedSingleton<double, true>();

      //surface!!!
      commandToBePosted = elevatorCommand(man_name, dep_v, timeout);

    }
    else if (latitude.domain().isSingleton() && longitude.domain().isSingleton()
        && depth.domain().isSingleton()) {
      double lat = latitude.domain().getTypedSingleton<double, true>();// latitude.domain().getSingleton());// atof(northing.domain().getStringSingleton().c_str());
      double lon = longitude.domain().getTypedSingleton<double, true>();
      double dep_v = depth.domain().getTypedSingleton<double, true>();//atof(depth.domain().getStringSingleton().c_str());
      double speed_mps = 1.5;
      long long duration = 0;

      if (secs.domain().isSingleton())
        duration = secs.domain().getTypedSingleton<long long, true>();

      if (speed.domain().isSingleton())
      {
        speed_mps = speed.domain().getTypedSingleton<double, true>();
      }
      else {
        //FIXME
        speed_mps = 1.5;
      }
      if (duration == 0)
        commandToBePosted = gotoCommand(man_name, lat, lon, dep_v, speed_mps, timeout);
      else if (secs.domain().isSingleton())
      {
        long long dur = secs.domain().getTypedSingleton<long long, true>();
        if (dep_v > 0)
          commandToBePosted = loiterCommand(man_name, lat, lon, dep_v, m_loiter_radius, speed_mps, (int)dur);

        else
          commandToBePosted = skeepingCommand(man_name, lat, lon, speed_mps, (int)dur);
      }

      //      Observation obs = Observation(*goal);
      //      obs.restrictAttribute("speed", FloatDomain(speed_mps));
      //      obs.restrictAttribute("eta", IntegerDomain(0, IntegerDomain::plus_inf));

    }
    else {
      syslog(log::error) << "variables are not singletons!\n";
      return;
    }
    next_goal = g.get()->object().str();
  }
}

void
Platform::handleRecall(goal_id const &g)
{
  syslog(log::warn) << "handleRecall(" << current_goal << ")";

  if (current_goal == g.get()->object().str())
  {
    IMC::Message * cmd = idleCommand("TREX recalled maneuver execution");
    sendMsg(*cmd);
    delete cmd;
  }
  else if (next_goal == g.get()->object().str())
  {
    IMC::Message * copy = commandToBePosted;
    next_goal = g.get()->object().str();
    delete commandToBePosted;
    commandToBePosted = NULL;
  }
}

void
Platform::convertToRelative(double lat, double lon, double &x, double &y)
{
  if (!aggregate.count(IMC::HomeRef::getIdStatic()))
    return;

  IMC::HomeRef * homeRef =
      dynamic_cast<IMC::HomeRef *>(aggregate[IMC::HomeRef::getIdStatic()]);
  WGS84::displacement(homeRef->lat, homeRef->lon, 0, lat, lon, 0, &x, &y);
}

void
Platform::convertToAbsolute(double northing, double easting, double &lat, double &lon)
{
  // double tmp = 0;
  if (lat == 0 && lon == 0) {
    if (!aggregate.count(IMC::HomeRef::getIdStatic()))
      return;

    IMC::HomeRef * homeRef = dynamic_cast<IMC::HomeRef *>(aggregate[IMC::HomeRef::getIdStatic()]);

    lat = homeRef->lat;
    lon = homeRef->lon;
  }

  WGS84::displace(northing, easting, &lat, &lon);
}

void
Platform::processState()
{

  /* Abort */

  if (received.count(IMC::Abort::getIdStatic()))
  {
    m_blocked = true;
  }

  /* ESTIMATED_STATE */

  if (received.count(IMC::EstimatedState::getIdStatic()))
  {
    IMC::EstimatedState * msg =
        dynamic_cast<IMC::EstimatedState *>(aggregate[IMC::EstimatedState::getIdStatic()]);

    Observation obs("estate", "Position");
    m_latitude = msg->lat;
    m_longitude = msg->lon;
    m_depth = msg->z;
    if (msg->ref == IMC::EstimatedState::RM_NED_LLD)
      WGS84::displace(msg->x, msg->y, &m_latitude, &m_longitude);

    obs.restrictAttribute("latitude", FloatDomain(m_latitude));
    obs.restrictAttribute("longitude", FloatDomain(m_longitude));
    obs.restrictAttribute("depth", FloatDomain(msg->z));


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
    postObservation(
        Observation("estate", "undefined"));
    estate_posted = true;
  }

  if (received.count(IMC::GpsFix::getIdStatic()))
  {
    IMC::GpsFix * fix = dynamic_cast<IMC::GpsFix *>(received[IMC::GpsFix::getIdStatic()]);

    if ((fix->validity & IMC::GpsFix::GFV_VALID_POS) != 0)
      uniqueObservation(Observation("gps", "Valid"));
    else
      uniqueObservation(Observation("gps", "Invalid"));

  }

  /* OPERATIONAL_LIMITS */

  if (received.count(IMC::OperationalLimits::getIdStatic()))
  {
    IMC::OperationalLimits * msg =
        dynamic_cast<IMC::OperationalLimits *>(received[IMC::OperationalLimits::getIdStatic()]);
    Observation obs("oplimits", "Limits");

    if (msg->mask & IMC::OperationalLimits::OPL_MAX_DEPTH)
      obs.restrictAttribute("maxDepth", FloatDomain(msg->max_depth));

    if ((msg->mask & IMC::OperationalLimits::OPL_MAX_ALT))
      obs.restrictAttribute("maxAltitude", FloatDomain(msg->max_depth));
    if(msg->mask & IMC::OperationalLimits::OPL_MIN_ALT)
      obs.restrictAttribute("minAltitude", FloatDomain(msg->min_altitude));

    if (msg->mask & IMC::OperationalLimits::OPL_MAX_SPEED)
      obs.restrictAttribute("maxSpeed", FloatDomain(msg->max_speed));
    if (msg->mask & IMC::OperationalLimits::OPL_MIN_SPEED)
      obs.restrictAttribute("minSpeed", FloatDomain(msg->min_speed));

    InsideOpLimits::set_oplimits(msg);

    oplimits_posted = true;
    uniqueObservation(obs);
  }
  else
  {
    if (!oplimits_posted) {
      uniqueObservation(Observation("oplimits", "undefined"));
      oplimits_posted = true;
    }
    if (!aggregate.count(IMC::OperationalLimits::getIdStatic()))
    {
      IMC::GetOperationalLimits get;
      sendMsg(get);
    }
  }

  /* VEHICLE_STATE */

  if ((postedCommand == NULL || tickWhenToPost > getCurrentTick()) && received.count(IMC::VehicleState::getIdStatic()))
  {
    IMC::VehicleState * msg =
        dynamic_cast<IMC::VehicleState *>(aggregate[IMC::VehicleState::getIdStatic()]);
    std::string mode;
    switch (msg->op_mode)
    {
      case IMC::VehicleState::VS_MANEUVER:
        mode = "Exec";
        break;
      case IMC::VehicleState::VS_SERVICE:
        mode = "Ready";
        break;
      case IMC::VehicleState::VS_CALIBRATION:
        mode = "Boot";
        break;
      case IMC::VehicleState::VS_EXTERNAL:
        mode = "Boot";
        break;
      case IMC::VehicleState::VS_ERROR:
        mode = "Error";
        break;
      default:
        std::ostringstream ss;
        ss << "Unknown vehicle mode: " << msg->op_mode;
        reportToDune(ss.str());
        syslog(log::warn) << ss.str();
        mode = "undefined";
        break;
    }

    if (last_vstate != msg->op_mode)
    {
      uniqueObservation(Observation("vstate", mode));
      if (msg->op_mode != IMC::VehicleState::VS_MANEUVER && !was_idle)
      {
        uniqueObservation(Observation("maneuver", "Idle"));
        uniqueObservation(Observation("command", "Idle"));
        was_idle = true;
      }
      else
        was_idle = false;
    }
    last_vstate = msg->op_mode;
  }
  else if (last_vstate == -1)
  {
    if (!was_idle)
    {
      uniqueObservation(Observation("vstate", "Boot"));
      uniqueObservation(Observation("maneuver", "Idle"));
      uniqueObservation(Observation("command", "Idle"));
      was_idle = true;
    }
  }

  /* VEHICLE_COMMAND */
  int eta = std::numeric_limits<int>::max();

  if (postedCommand != NULL && tickWhenToPost <= getCurrentTick())
  {
    IMC::PlanControl * posted =  dynamic_cast<IMC::PlanControl*>(postedCommand);
    IMC::Message * maneuver = posted->arg.get()->clone();

    Observation obs = maneuverObservation(maneuver);
    postObservation(obs, true);
    postedObservations[obs.object().str()].reset(new Observation(obs));
    uniqueObservation(Observation("vstate", "Exec"));
    delete postedCommand;
    postedCommand = NULL;
  }
  else if (received.count(IMC::VehicleState::getIdStatic()))
  {
    IMC::VehicleState *lastState =
        dynamic_cast<IMC::VehicleState*>(aggregate[IMC::VehicleState::getIdStatic()]);
    eta = lastState->maneuver_eta;
    if ((lastState->op_mode == IMC::VehicleState::VS_MANEUVER
        && !lastCommand.maneuver.isNull()))
    {

      IMC::Message * man = NULL;
      man = lastCommand.maneuver.get();
      Observation obs = maneuverObservation(man);
      obs.restrictAttribute("eta", IntegerDomain(eta));
      lastCommand.maneuver = IMC::InlineMessage();
      uniqueObservation(obs);
    }
  }
}


Observation Platform::maneuverObservation(IMC::Message * man)
{
  Observation obs("command", "Maneuver");

  if (man->getId() == IMC::Goto::getIdStatic())
  {
    IMC::Goto * m = dynamic_cast<IMC::Goto *>(man);
    obs.restrictAttribute("latitude", FloatDomain(m->lat /*floor(n,2), ceil(n,2)*/));
    obs.restrictAttribute("longitude", FloatDomain(m->lon /*floor(e,2), ceil(e,2)*/));
    obs.restrictAttribute("depth", FloatDomain(m->z));
    //  obs.restrictAttribute("type", EnumDomain("goto"));

    if (m->speed_units == IMC::Goto::SUNITS_METERS_PS)
    {
      obs.restrictAttribute("speed", FloatDomain(m->speed));
    }
    else if (m->speed_units == IMC::Goto::SUNITS_RPM)
    {
      obs.restrictAttribute("speed", FloatDomain(m->speed/m_rpm_speed_factor));
    }
    obs.restrictAttribute("secs", IntegerDomain(0));
  }
  else if (man->getId() == IMC::Loiter::getIdStatic())
  {

    IMC::Loiter * m = dynamic_cast<IMC::Loiter *>(man);
    //double n, e;
    obs.restrictAttribute("latitude", FloatDomain(m->lat /*floor(n,2), ceil(n,2)*/));
    obs.restrictAttribute("longitude", FloatDomain(m->lon /*floor(e,2), ceil(e,2)*/));
    obs.restrictAttribute("depth", FloatDomain(m->z));
    if (m->speed_units == IMC::Loiter::SUNITS_METERS_PS)
    {
      obs.restrictAttribute("speed", FloatDomain(m->speed));
    }
    else if (m->speed_units == IMC::Loiter::SUNITS_RPM)
    {
      obs.restrictAttribute("speed", FloatDomain(m->speed/m_rpm_speed_factor));
    }

    obs.restrictAttribute("secs", IntegerDomain(m->duration));
    //obs.restrictAttribute("type", EnumDomain("loiter"));
  }
  else if (man->getId() == IMC::StationKeeping::getIdStatic())
  {
    IMC::StationKeeping * m = dynamic_cast<IMC::StationKeeping *>(man);
    //double n, e;
    obs.restrictAttribute("latitude", FloatDomain(m->lat /*floor(n,2), ceil(n,2)*/));
    obs.restrictAttribute("longitude", FloatDomain(m->lon /*floor(e,2), ceil(e,2)*/));
    obs.restrictAttribute("depth", FloatDomain(0));
    //obs.restrictAttribute("type", EnumDomain("skeeping"));
    if (m->speed_units == IMC::StationKeeping::SUNITS_METERS_PS)
    {
      obs.restrictAttribute("speed", FloatDomain(m->speed));
    }
    else if (m->speed_units == IMC::StationKeeping::SUNITS_RPM)
    {
      obs.restrictAttribute("speed", FloatDomain(m->speed/m_rpm_speed_factor));
    }
    obs.restrictAttribute("secs", IntegerDomain(m->duration));
  }
  else if (man->getId() == IMC::Teleoperation::getIdStatic())
  {
    //  obs.restrictAttribute("type", TREX::europa::EuropaEntity("teleop"));
    IMC::EstimatedState * msg =
        dynamic_cast<IMC::EstimatedState *>(aggregate[IMC::EstimatedState::getIdStatic()]);
    //double lat = msg->lat, lon = msg->lon, hae = 0.0, northing, easting;
    obs.restrictAttribute("latitude", FloatDomain(msg->lat /*floor(n,2), ceil(n,2)*/));
    obs.restrictAttribute("longitude", FloatDomain(msg->lon /*floor(e,2), ceil(e,2)*/));
    obs.restrictAttribute("depth", FloatDomain(0));
    obs.restrictAttribute("speed", FloatDomain(msg->v));
    obs.restrictAttribute("secs", IntegerDomain(0));
    //obs.restrictAttribute("type", EnumDomain("teleop"));
  }
  else if (man->getId() == IMC::Elevator::getIdStatic())
  {
    IMC::Elevator * msg =
        dynamic_cast<IMC::Elevator *>(man);
    obs.restrictAttribute("latitude", FloatDomain(msg->lat));
    obs.restrictAttribute("longitude", FloatDomain(msg->lon ));
    obs.restrictAttribute("depth", FloatDomain(msg->end_z));
    obs.restrictAttribute("secs", IntegerDomain(0));
  }

  return obs;
}

bool
Platform::sendMsg(Message& msg, Address &dest)
{
  Dune::Utils::ByteBuffer bb;
  try
  {
    msg.setTimeStamp();
    msg.serialize(bb);
    send.write((const char*)bb.getBuffer(), msg.getSerializationSize(), dest,
               duneport);
  }
  catch (std::runtime_error& e)
  {
    syslog("ERROR", log::error)<< e.what();
    return false;
  }
  return true;
}

bool
Platform::sendMsg(Message& msg, std::string ip, int port)
{
  Dune::Utils::ByteBuffer bb;
  try
  {
    msg.setTimeStamp();
    msg.setSource(65000);
    msg.setDestination(remote_id);
    msg.serialize(bb);

    if (debug)
      msg.toText(syslog("debug") << "sending message:\n");

    m_mutex.lock();
    send.write((const char*)bb.getBuffer(), msg.getSerializationSize(),
               Address(ip.c_str()), port);
    m_mutex.unlock();
  }
  catch (std::runtime_error& e)
  {
    syslog("ERROR", log::error)<< e.what();
    return false;
  }
  return true;
}

bool
Platform::sendMsg(Message& msg)
{
  return sendMsg(msg, duneip, duneport);
}

IMC::Message * Platform::getManeuverCommand(const std::string &man_name, IMC::Message * maneuver)
{
  IMC::PlanControl * pcontrol = new IMC::PlanControl();

  pcontrol->arg.set(maneuver);
  pcontrol->info = "Maneuver initiated from TREX";
  pcontrol->op = IMC::PlanControl::PC_START;
  pcontrol->type = IMC::PlanControl::PC_REQUEST;
  pcontrol->plan_id = man_name;
  pcontrol->flags = 0;

  return pcontrol;
}

bool Platform::reportToDune(int type, const std::string &message)
{
  return reportToDune(IMC::LogBookEntry::LBET_INFO, "Autonomy.TREX", message);
}

bool Platform::reportToDune(const std::string &message)
{
  return reportToDune(IMC::LogBookEntry::LBET_INFO, message);
}

bool Platform::reportToDune(int type,  const std::string &context, const std::string &text)
{
  IMC::LogBookEntry entry;
  entry.text = text;
  entry.context = context;
  entry.htime = Time::Clock::getSinceEpoch();
  entry.type = type;
  return sendMsg(entry);
}

bool Platform::reportErrorToDune(const std::string &message)
{
  return reportToDune(IMC::LogBookEntry::LBET_ERROR, message);
}

IMC::Message *
Platform::gotoCommand(const std::string &man_name, double lat, double lon, double depth, double speed, boost::optional<long long> timeout)
{
  IMC::Goto * msg = new IMC::Goto();

  msg->lat = lat;
  msg->lon = lon;
  msg->z = depth;

  if (m_use_rpm)
  {
    msg->speed = speed * m_rpm_speed_factor;
    msg->speed_units = IMC::Goto::SUNITS_RPM;
  }
  else
  {
    msg->speed = speed;
    msg->speed_units = IMC::Goto::SUNITS_METERS_PS;
  }

  if (timeout)
    msg->timeout = *timeout;

  return getManeuverCommand(man_name, msg);
}

IMC::Message *
Platform::elevatorCommand(const std::string &man_name, double target_depth, boost::optional<long long> timeout)
{
  IMC::Elevator * msg = new IMC::Elevator();

  IMC::EstimatedState *state = dynamic_cast<IMC::EstimatedState*>(aggregate[IMC::EstimatedState::getIdStatic()]);



  msg->lat = state->lat;
  msg->lon = state->lon;
  msg->end_z = target_depth;

  if (state->ref == IMC::EstimatedState::RM_NED_LLD)
    WGS84::displace(state->x, state->y, &msg->lat, &msg->lon);

  msg->flags = IMC::Elevator::FLG_CURR_POS;

  msg->speed = 1000;
  msg->speed_units = IMC::Elevator::SUNITS_RPM;

  msg->pitch = Angles::radians(15);
  msg->radius = m_loiter_radius;



  if (timeout)
    msg->timeout = *timeout;

  return getManeuverCommand(man_name, msg);
}

IMC::Message *
Platform::elevatorCommand(const std::string &man_name, double lat, double lon, double target_depth, double speed, boost::optional<long long> timeout)
{
  IMC::Elevator * msg = new IMC::Elevator();

  msg->lat = lat;
  msg->lon = lon;
  msg->end_z = target_depth;

  if (m_use_rpm)
  {
    msg->speed = speed * m_rpm_speed_factor;
    msg->speed_units = IMC::Elevator::SUNITS_RPM;
  }
  else
  {
    msg->speed = speed;
    msg->speed_units = IMC::Elevator::SUNITS_METERS_PS;
  }

  msg->pitch = Angles::radians(15);
  msg->radius = m_loiter_radius;

  if (timeout)
    msg->timeout = *timeout;

  return getManeuverCommand(man_name, msg);
}

IMC::Message *
Platform::skeepingCommand(const std::string &man_name, double lat, double lon, double speed,
    int seconds)
{
  IMC::StationKeeping * msg = new IMC::StationKeeping();

  msg->lat = lat;
  msg->lon = lon;
  msg->duration = seconds;
  msg->radius = m_skeeping_radius;
  if (m_use_rpm)
  {
    msg->speed = speed * m_rpm_speed_factor;
    msg->speed_units = IMC::Goto::SUNITS_RPM;
  }
  else
  {
    msg->speed = speed;
    msg->speed_units = IMC::Goto::SUNITS_METERS_PS;
  }

  return getManeuverCommand(man_name, msg);
}

IMC::Message *
Platform::loiterCommand(const std::string &man_name, double lat, double lon, double depth, double radius,
    double speed, int seconds)
{
  IMC::Loiter * msg = new IMC::Loiter();
  IMC::Message * ret;

  msg->lat = lat;
  msg->lon = lon;
  msg->z = depth;
  msg->duration = seconds;
  msg->radius = m_loiter_radius;
  msg->direction = IMC::Loiter::LD_CLOCKW;
  if (m_use_rpm)
  {
    msg->speed = speed * m_rpm_speed_factor;
    msg->speed_units = IMC::Goto::SUNITS_RPM;
  }
  else
  {
    msg->speed = speed;
    msg->speed_units = IMC::Goto::SUNITS_METERS_PS;
  }

  return getManeuverCommand(man_name, msg);
}

IMC::Message *
Platform::idleCommand(const std::string &man_name)
{
  IMC::PlanControl * pcontrol = new IMC::PlanControl();

  pcontrol->info = "Idle request by TREX";
  pcontrol->op = IMC::PlanControl::PC_STOP;
  pcontrol->type = IMC::PlanControl::PC_REQUEST;
  pcontrol->flags = 0;

  return pcontrol;
}

/*
 * class TREX::LSTS::Platform::log_proxy
 */

// structors 

Platform::log_proxy::~log_proxy() {
  if( this==m_platform->m_active_proxy ) {
    m_platform->m_active_proxy = NULL;
  }
}

// callback

void Platform::log_proxy::operator()(log::entry::pointer msg) {
  m_platform->m_active_proxy = this;
  // Example that display error/warnings on std::cerr

  std::ostringstream ss;
  if( msg->is_dated() )
    ss<<'@'<<msg->date()<<' ';

  ss<<msg->content();

  if( log::warn==msg->kind() )
    m_platform->reportToDune(IMC::LogBookEntry::LBET_WARNING, who.str(), ss.str());
  else if ( log::error==msg->kind() )
    m_platform->reportToDune(IMC::LogBookEntry::LBET_ERROR, who.str(), ss.str());
  else if (TeleoReactor::obs==msg->kind() )
    m_platform->reportToDune(IMC::LogBookEntry::LBET_INFO, who.str(), ss.str());
}


