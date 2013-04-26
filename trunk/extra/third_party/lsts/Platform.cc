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
    //ControlInterface * Platform::controlInterfaceInstance = 0;
    ImcAdapter m_adapter;
    Reference m_ref;

    Platform::Platform(TeleoReactor::xml_arg_type arg) :
                    TeleoReactor(arg, false)
    {
      m_firstTick = true;
      m_blocked = false;
      m_connected = true;
      // connect with Safety bug through a singleton object
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
   //   provide("imu");
   //   provide("lbl");
   //   provide("sidescan");
   //   provide("multibeam");
   //   provide("camera");

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
          if (m_connected)
            std::cerr <<"Disconnected from DUNE\n";

          syslog(log::warn) << "Disconnected from DUNE";
          m_connected = false;
        }

        else
        {
          syslog(log::info) << "Received a total of " << msg_count << " messages\n";
          if (!m_connected)
            std::cerr <<"Now connected to DUNE\n";
          m_connected = true;
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

      std::cerr << "handleRequest(" << gpred << ")";

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
    Platform::handleTrexOperation(TrexOperation trexOp)
    {
      switch (trexOp.op)
      {
        case TrexOperation::OP_POST_GOAL:
          postGoalToken(trexOp.goal_id, TrexToken(*trexOp.token.get()));
          break;
      }

    }

    void
    Platform::postGoalToken(std::string goald_id, TrexToken token)
    {
      std::stringstream ss;
      std::string timeline = token.timeline;
      std::string predicate = token.predicate;

      ss << "<Goal on='" << token.timeline << "' pred='"
          << token.predicate << "' id='" << goald_id << "'>\n";

      MessageList<TrexAttribute>::const_iterator it;
      for (it = token.attributes.begin(); it != token.attributes.end(); it++)
      {
        ss << "\t<Variable name='" << (*it)->name << "'>\n";

        switch ((*it)->attr_type)
        {
          case TrexAttribute::TYPE_FLOAT:
            ss << "\t\t<float min='" << (*it)->min << "' max='" << (*it)->max << "'/>\n";
            break;
          case TrexAttribute::TYPE_INT:
            ss << "\t\t<int min='" << (*it)->min << "' max='" << (*it)->max << "'/>\n";
            break;
          case TrexAttribute::TYPE_STRING:
            ss << "\t\t<string min='" << (*it)->min << "' max='" << (*it)->max << "'/>\n";
            break;
          case TrexAttribute::TYPE_BOOL:
            ss << "\t\t<bool min='" << (*it)->min << "' max='" << (*it)->max << "'/>\n";
            break;
          case TrexAttribute::TYPE_ENUM:
            ss << "\t\t<enum min='" << (*it)->min << "' max='" << (*it)->max << "'/>\n";
            break;
          default:
            std::cerr << "Error parsing attribute: ";
            (*it)->toText(std::cerr);
            break;
        }

        ss << "\t</Variable>\n";

        if(m_env->getControlInterfaceReactor() != NULL)
        {
          m_env->getControlInterfaceReactor()->proccess_message(ss.str());
        }
        else
        {
          std::cerr << "ControlInterface not instantiated!\n";
        }
      }
    }

    void
    Platform::processState()
    {

      // if DUNE is disconnected everything is on initial (unknown / boot) state...
      if (!m_connected)
      {
        m_ref = Reference();
        received.clear();
      }

      PlanControlState * pcstate =
          dynamic_cast<IMC::PlanControlState *>(received[PlanControlState::getIdStatic()]);
      postUniqueObservation(m_adapter.planControlStateObservation(pcstate));
      if (pcstate != NULL)
        m_blocked = !(pcstate->state == PlanControlState::PCS_EXECUTING);


      // Translate incoming messages into observations
      EstimatedState * estate =
          dynamic_cast<EstimatedState *>(received[EstimatedState::getIdStatic()]);
      postUniqueObservation(m_adapter.estimatedStateObservation(estate));

      VehicleMedium * medium =
          dynamic_cast<VehicleMedium *>(received[VehicleMedium::getIdStatic()]);
      postUniqueObservation(m_adapter.vehicleMediumObservation(medium));

      if (!m_blocked && m_ref.flags == 0 && estate != NULL)
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
        if (m_ref.lat != 0 || m_ref.lon != 0)
          sendMsg(m_ref);
      }

      FollowRefState * frefstate =
          dynamic_cast<IMC::FollowRefState *>(received[FollowRefState::getIdStatic()]);
        postUniqueObservation(m_adapter.followRefStateObservation(frefstate));

      OperationalLimits * oplims =
          dynamic_cast<IMC::OperationalLimits *>(received[OperationalLimits::getIdStatic()]);
      postUniqueObservation(m_adapter.opLimitsObservation(oplims));

      // Operational limits are sent by DUNE on request
      if (oplims == NULL)
      {
        GetOperationalLimits req;
        sendMsg(req);
      }

      TrexOperation * command = dynamic_cast<TrexOperation*>(received[TrexOperation::getIdStatic()]);
      if (command != NULL)
      {
        handleTrexOperation(*command);
        delete received[TrexOperation::getIdStatic()];
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


