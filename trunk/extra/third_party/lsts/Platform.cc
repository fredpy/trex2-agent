/** @file "Platform.cc"
 * @brief Platform plugin implementation
 *
 * @author Jose Pinto <zepinto@gmail.com>
 * @ingroup lsts
 */

# include "Platform.hh"

namespace
{

  /** @brief TREX log entry point */
  SingletonUse<LogManager> s_log;

  /** @brief Platform reactor declaration */
  TeleoReactor::xml_factory::declare<TREX::LSTS::Platform> decl("Platform");

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

  namespace LSTS
  {
    bool was_idle = false;
    int remote_id = 0;
    ControlInterface * Platform::controlInterfaceInstance = 0;
    ImcAdapter m_adapter;
    Reference m_ref;

    Platform::Platform(TeleoReactor::xml_arg_type arg) :
                TeleoReactor(arg, false)
    {
      m_firstTick = true;
      m_blocked = false;
      m_env->setPlatformReactor(this);
      duneport = parse_attr<int>(6002, TeleoReactor::xml_factory::node(arg),
                                 "duneport");
      duneip = parse_attr<std::string>("127.0.0.1",
                                       TeleoReactor::xml_factory::node(arg),
                                       "duneip");
      debug = parse_attr<bool>(false, TeleoReactor::xml_factory::node(arg),
                               "debug");
      localport = parse_attr<int>(false, TeleoReactor::xml_factory::node(arg),
                                  "localport");

      // Timelines for posting observations from DUNE
      provide("estate", false);
      provide("medium", false);
      provide("control", false);
      provide("oplimits", false);

      // Timelines that can be controlled by other reactors
      provide("reference");
      provide("payload");

      bfr = new uint8_t[65535];
    }

    void
    Platform::handleInit()
    {
      syslog(log::info) << "Connecting to dune on " << duneip << ":" << duneport;
      receive.bind(localport, Address::Any, true);
      receive.addToPoll(iom);

      syslog(log::info) << "listening on port " << localport << "...";
    }

    void
    Platform::handleTickStart()
    {
      try
      {
        Address addr;
        int msg_count = 0;

        while (iom.poll(0))
        {
          msg_count++;
          uint16_t rv = receive.read((char*)bfr, 65535, &addr);
          IMC::Message * msg = IMC::Packet::deserialize(bfr, rv);
          if (remote_id == 0)
            remote_id = msg->getSource();

          // substitute previously received message
          if (received.count(msg->getId()))
            delete received[msg->getId()];
          received[msg->getId()] = msg;
        }

        if (msg_count < 1)
        {
          syslog(log::warn) << "No messages from DUNE!\n";
        }

        else
        {
          syslog(log::info) << "Received a total of " << msg_count << " messages\n";
        }
      }
      catch (std::runtime_error& e)
      {
        syslog(log::error) << "Error during message processing: " << e.what();
        std::cerr << e.what();
      }
    }

    bool
    Platform::synchronize()
    {
      processState();

      Heartbeat hb;
      sendMsg(hb);
      return true;
    }

    void
    Platform::handleRequest(goal_id const &g)
    {

      Goal * goal = g.get();

      std::string gname = (goal->object()).str();
      std::string gpred = (goal->predicate()).str();
      std::string man_name;

      if (gname == "reference" && gpred == "Going")
        handleGoingRequest(*goal);
    }

    void
    Platform::handleRecall(goal_id const &g)
    {

    }

    bool
    Platform::postUniqueObservation(TREX::transaction::Observation obs)
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
    Platform::setControlInterface(ControlInterface * itf)
    {
      controlInterfaceInstance = itf;
    }

    void
    Platform::processState()
    {

      // Translate incoming messages into observations
      EstimatedState * estate =
          dynamic_cast<EstimatedState *>(received[EstimatedState::getIdStatic()]);
      postUniqueObservation(m_adapter.estimatedStateObservation(estate));

//      IMC::GpsFix * fix =
//          dynamic_cast<IMC::GpsFix *>(received[IMC::GpsFix::getIdStatic()]);
//      postUniqueObservation(m_adapter.gpsFixObservation(fix));

      VehicleMedium * medium =
          dynamic_cast<VehicleMedium *>(received[VehicleMedium::getIdStatic()]);
      postUniqueObservation(m_adapter.vehicleMediumObservation(medium));
      FollowRefState * frefstate =
          dynamic_cast<IMC::FollowRefState *>(received[FollowRefState::getIdStatic()]);
      postUniqueObservation(m_adapter.followRefStateObservation(frefstate));

      PlanControlState * pcstate =
          dynamic_cast<IMC::PlanControlState *>(received[PlanControlState::getIdStatic()]);
      postUniqueObservation(m_adapter.planControlStateObservation(pcstate));

      OperationalLimits * oplims =
          dynamic_cast<IMC::OperationalLimits *>(received[OperationalLimits::getIdStatic()]);
      postUniqueObservation(m_adapter.opLimitsObservation(oplims));

      TrexCommand * command = dynamic_cast<TrexCommand*>(received[TrexCommand::getIdStatic()]);

      if (command != NULL)
      {
      switch (command->command)
      {
        case TrexCommand::OP_POST_GOAL:
          syslog(log::info) << "received (" << command->goal_id << "): " << command->goal_xml;
          if (controlInterfaceInstance) {
            controlInterfaceInstance->proccess_message(command->goal_xml);
          }
          break;
        case TrexCommand::OP_ENABLE:
          syslog(log::warn) << "Enable TREX command received";
          // post active observation ...
          postUniqueObservation(Observation("supervision", "Active"));
          m_blocked = false;

          break;
        case TrexCommand::OP_DISABLE:
          syslog(log::warn) << "Disable TREX command received";
          // post blocked observation ...
          postUniqueObservation(Observation("supervision", "Blocked"));
          m_blocked = true;
          break;
      }
      }

      if (pcstate != NULL)
        m_blocked = !(pcstate->state == PlanControlState::PCS_EXECUTING);

      // Operational limits are sent by DUNE on request
      if (oplims == NULL)
      {
        GetOperationalLimits req;
        sendMsg(req);
      }

      if (m_ref.flags == 0 && estate != NULL)
      {
        m_ref.flags = Reference::FLAG_LOCATION | Reference::FLAG_Z;
        m_ref.lat = estate->lat;
        m_ref.lon = estate->lon;
        WGS84::displace(estate->x, estate->y, &(m_ref.lat), &(m_ref.lon));
        DesiredZ desZ;
        desZ.value = 0;
        desZ.z_units = Z_DEPTH;
        m_ref.z.set(desZ);
      }

      // Send current reference to DUNE
      if (!m_blocked)
      {
        sendMsg(m_ref);
      }



    }

    void
    Platform::handleGoingRequest(Goal g) {
      Variable v;
      v = g.getAttribute("latitude");
      int flags = Reference::FLAG_LOCATION;

      if (v.domain().isSingleton())
        m_ref.lat = v.domain().getTypedSingleton<double, true>();

      v = g.getAttribute("longitude");
      if (v.domain().isSingleton())
        m_ref.lon = v.domain().getTypedSingleton<double, true>();

      v = g.getAttribute("z");
      if (v.domain().isSingleton())
      {
        double z = v.domain().getTypedSingleton<double, true>();
        DesiredZ desZ;
        flags |= Reference::FLAG_LOCATION;

        if (z >= 0)
        {
          desZ.value = z;
          desZ.z_units = Z_DEPTH;
        }
        else
        {
          desZ.value = -z;
          desZ.z_units = Z_ALTITUDE;
        }
        m_ref.z.set(desZ);
      }

      v = g.getAttribute("speed");
      if (v.domain().isSingleton())
      {
        double speed = v.domain().getTypedSingleton<double, true>();
        DesiredSpeed desSpeed;
        flags |= Reference::FLAG_SPEED;
        desSpeed.value = speed;
        desSpeed.speed_units = SUNITS_METERS_PS;
        m_ref.speed.set(desSpeed);
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
        msg.setSource(TREX_ID);
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

    Platform::~Platform()
    {
      m_env->setPlatformReactor(NULL);
      if (NULL != bfr)
        delete[] bfr;
    }
  }
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

