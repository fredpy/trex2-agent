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
                  LstsReactor(arg)
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
      //m_links = std::map<std::string, Announce*>();

      // Timelines for posting observations from DUNE
      provide("estate", false);
      provide("medium", false);
      provide("control", false);
      provide("oplimits", false);
      provide("refstate", false);

      // Timelines that can be controlled by other reactors
      provide("reference");

    }


    void
    Platform::handleInit()
    {
      syslog(log::info) << "Connecting to dune on " << duneip << ":" << duneport;
      m_adapter.bind(localport);
      syslog(log::info) << "listening on port " << localport << "...";
    }

    void
    Platform::handleTickStart()
    {


      if (!m_blocked && !m_goals_pending.empty())
      {
        goal_id goal = m_goals_pending.front();
        std::string gname = (goal->object()).str();
        std::string gpred = (goal->predicate()).str();
        std::string man_name;
        bool commited = false;
        if (gname == "reference" && gpred == "Going")
          commited = handleGoingRequest(goal);
        else if (gname == "reference" && gpred == "At")
          commited = handleAtRequest(goal);

        if (commited)
        {
          m_goals_pending.remove(goal);
          postUniqueObservation(*goal.get());
        }
      }

      Announce * ann;
      try
      {
        Address addr;
        int msg_count = 0;

        IMC::Message * msg;

        while ((msg = m_adapter.poll(0, false)) != NULL)
        {
          msg_count++;
          if (remote_id == 0)
            remote_id = msg->getSource();

          if (msg->getId() == Announce::getIdStatic())
          {
            ann = (Announce *) dynamic_cast<Announce *>(msg);

            if (m_receivedAnnounces[ann->sys_name] != NULL)
              delete m_receivedAnnounces[ann->sys_name];
            m_receivedAnnounces[ann->sys_name] = ann;
          }
          if (msg->getId() == TrexOperation::getIdStatic())
          {
            TrexOperation * command = dynamic_cast<TrexOperation*>(msg);
            handleTrexOperation(*command);
          }
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

      std::cerr << "handleRequest(" << g << ", " << *goal << ")" << std::endl;

      m_goals_pending.push_back(g);
    }

    void
    Platform::handleRecall(goal_id const &g)
    {
      Goal * goal = g.get();

      std::string gname = (goal->object()).str();
      std::string gpred = (goal->predicate()).str();
      std::string man_name;
      std::cerr << "handleRecall(" << g << ", " << *goal << ")" << std::endl;

      m_goals_pending.remove(g);

      if (gname == "reference" && gpred == "Going")
        handleGoingRecall(g);
    }

    void
    Platform::handleTrexOperation(TrexOperation trexOp)
    {
      switch (trexOp.op)
      {
        case TrexOperation::OP_POST_GOAL:
          postGoalToken(trexOp.goal_id, TrexToken(*trexOp.token.get()));
          break;
        case TrexOperation::OP_POST_TOKEN:
          postObservationToken(TrexToken(*trexOp.token.get()));
          break;
      }
    }

    void
    Platform::postObservationToken(TrexToken token)
    {
      obs_map::iterator it = postedObservations.find(token.timeline);

      // If no such timeline has ever been posted, a new timeline will now be provided
      if (it == postedObservations.end())
        provide(token.timeline, false, false);
      //Observation obs = m_adapter.genericObservation(&token);
      //std::cerr << "Posting observation: " << obs << "\n";
      //postUniqueObservation(obs);
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

      }
      ss << "\t</Goal>\n";
      std::cerr << "Received goal:\n" << ss.str();

      if(m_env->getControlInterfaceReactor() != NULL)
      {
        m_env->getControlInterfaceReactor()->proccess_message(ss.str());
      }
      else
      {
        std::cerr << "ControlInterface not instantiated!\n";
      }
    }

    void
    Platform::processState()
    {

      bool createNewReference = false;

      // if DUNE is disconnected everything is on initial (unknown / boot) state...
      if (!m_connected)
      {
        received.clear();
      }

      PlanControlState * pcstate =
          dynamic_cast<IMC::PlanControlState *>(received[PlanControlState::getIdStatic()]);
      postUniqueObservation(m_adapter.planControlStateObservation(pcstate));
      if (pcstate != NULL)
        m_blocked = !(pcstate->state == PlanControlState::PCS_EXECUTING)
        && pcstate->plan_id == "trex_plan";

      // Translate incoming messages into observations
      EstimatedState * estate =
          dynamic_cast<EstimatedState *>(received[EstimatedState::getIdStatic()]);
      postUniqueObservation(m_adapter.estimatedStateObservation(estate));

      VehicleMedium * medium =
          dynamic_cast<VehicleMedium *>(received[VehicleMedium::getIdStatic()]);
      postUniqueObservation(m_adapter.vehicleMediumObservation(medium));

      FollowRefState * frefstate =
          dynamic_cast<IMC::FollowRefState *>(received[FollowRefState::getIdStatic()]);
      postUniqueObservation(m_adapter.followRefStateObservation(frefstate));

      OperationalLimits * oplims =
          dynamic_cast<IMC::OperationalLimits *>(received[OperationalLimits::getIdStatic()]);
      postUniqueObservation(m_adapter.opLimitsObservation(oplims));

      std::map<std::string, Announce *>::iterator it;

      for (it = m_receivedAnnounces.begin(); it != m_receivedAnnounces.end(); it++)
      {
        Observation obs = m_adapter.announceObservation(it->second);
        postUniqueObservation(obs);
      }

      if (frefstate != NULL && frefstate->state == FollowRefState::FR_WAIT)
        createNewReference = true;

      if (m_ref.flags == 0)
        createNewReference = true;

      if (createNewReference && estate != NULL)
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
        //m_ref.toText(std::cout);
      }

      if (atDestination(frefstate))
      {
        Observation obs("reference", "At");

        obs.restrictAttribute("latitude", FloatDomain(m_ref.lat));
        obs.restrictAttribute("longitude", FloatDomain(m_ref.lon));

        if (!m_ref.z.isNull())
        {
          switch(m_ref.z->z_units)
          {
            case (Z_DEPTH):
                          obs.restrictAttribute("z", FloatDomain(m_ref.z->value));
            break;
            case (Z_ALTITUDE):
                          obs.restrictAttribute("z", FloatDomain(-m_ref.z->value));
            break;
            case (Z_HEIGHT):
                          obs.restrictAttribute("z", FloatDomain(m_ref.z->value));
            break;
            default:
              break;
          }
        }

        if (!m_ref.speed.isNull())
        {
          obs.restrictAttribute("speed",
                                FloatDomain((m_ref.speed->value)));
        }
        postUniqueObservation(obs);
      }

      // Operational limits are sent by DUNE on request
      if (oplims == NULL)
      {
        GetOperationalLimits req;
        sendMsg(req);
      }
    }

    void Platform::handleGoingRecall(goal_id const &g)
    {
      EstimatedState * estate =
          dynamic_cast<EstimatedState *>(received[EstimatedState::getIdStatic()]);
      if (estate != NULL)
      {
        double latitude, longitude;
        latitude = estate->lat;
        longitude = estate->lon;
        WGS84::displace(estate->x, estate->y, &latitude, &longitude);
        int flags = Reference::FLAG_LOCATION;
        DesiredZ dz;
        if (estate->depth != -1)
        {
          flags |= Reference::FLAG_Z;
          dz.value = estate->depth;
          dz.z_units = Z_DEPTH;
        }
        else if (estate->alt != -1)
        {
          flags |= Reference::FLAG_Z;
          dz.value = estate->alt;
          dz.z_units = Z_ALTITUDE;
        }
        else if (estate->height != -1){
          flags |= Reference::FLAG_Z;
          dz.value = estate->height;
          dz.z_units = Z_HEIGHT;
        }
        m_ref.flags = flags;
        m_ref.z.set(dz);
        m_ref.lat = latitude;
        m_ref.lon = longitude;
      }
    }

    bool
    Platform::handleAtRequest(goal_id const &goal)
    {

      Goal *g = goal.get();
      double my_lat = 0, my_lon = 0, my_z = 0, req_lat = 0, req_lon = 0, req_z = 0;

      if(g->getAttribute("latitude").domain().isSingleton())
      {
        req_lat = g->getAttribute("latitude").domain().getTypedSingleton<double, true>();
      }
      if(g->getAttribute("longitude").domain().isSingleton())
      {
        req_lon = g->getAttribute("longitude").domain().getTypedSingleton<double, true>();
      }
      if(g->getAttribute("z").domain().isSingleton())
      {
        req_z = g->getAttribute("z").domain().getTypedSingleton<double, true>();
      }

      EstimatedState * estate =
          dynamic_cast<EstimatedState *>(received[EstimatedState::getIdStatic()]);
      if (estate != NULL)
      {
        my_lat = estate->lat;
        my_lon = estate->lon;
        if (req_z >= 0)
          my_z = estate->depth;
        else
          my_z = estate->alt;

        WGS84::displace(estate->x, estate->y, &my_lat, &my_lon);
      }

      double dist = WGS84::distance(my_lat, my_lon, 0, req_lat, req_lon, 0);

      if (dist < 3)
        return true;
      else
        return false;
    }

    bool
    Platform::handleYoYoRequest(goal_id const &goal)
    {
      return handleGoingRequest(goal);
    }

    bool
    Platform::handleGoingRequest(goal_id const &goal)
    {

      Goal *g = goal.get();
      Variable v;
      v = g->getAttribute("latitude");

      int flags = Reference::FLAG_LOCATION;

      if (v.domain().isSingleton())
        m_ref.lat = v.domain().getTypedSingleton<double, true>();

      v = g->getAttribute("longitude");
      if (v.domain().isSingleton())
        m_ref.lon = v.domain().getTypedSingleton<double, true>();

      v = g->getAttribute("z");
      if (v.domain().isSingleton())
      {
        double z = v.domain().getTypedSingleton<double, true>();
        DesiredZ desZ;
        flags |= Reference::FLAG_Z;

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

      m_ref.flags = flags;

      v = g->getAttribute("speed");
      if (v.domain().isSingleton())
      {
        double speed = v.domain().getTypedSingleton<double, true>();
        DesiredSpeed desSpeed;
        flags |= Reference::FLAG_SPEED;
        desSpeed.value = speed;
        desSpeed.speed_units = SUNITS_METERS_PS;
        m_ref.speed.set(desSpeed);
      }

      sendMsg(m_ref);

      return true;
    }

    bool
    Platform::atDestination(FollowRefState * frefstate)
    {
      if (frefstate == NULL)
        return false;

      if ((frefstate->proximity & FollowRefState::PROX_XY_NEAR) == 0)
        return false;

      if ((frefstate->proximity & FollowRefState::PROX_Z_NEAR) == 0)
        return false;

      if (!sameReference(&m_ref, frefstate->reference.get()))
        return false;

      return true;
    }

    bool
    Platform::sameReference(const IMC::Reference *msg1, const IMC::Reference *msg2)
    {
      if (msg1->flags != msg2->flags)
        return false;
      if (msg1->lat != msg2->lat)
        return false;
      if (msg1->lon != msg2->lon)
        return false;

      if (msg1->z.isNull() != msg2->z.isNull())
        return false;
      else if (!msg1->z.isNull())
      {
        const IMC::DesiredZ *z1 = msg1->z.get();
        const IMC::DesiredZ *z2 = msg2->z.get();

        if (!z1->fieldsEqual(*z2))
          return false;
      }

      if (msg1->speed.isNull() != msg2->speed.isNull())
        return false;
      else if (!msg1->speed.isNull())
      {
        const IMC::DesiredSpeed *s1 = msg1->speed.get();
        const IMC::DesiredSpeed *s2 = msg2->speed.get();

        if (!s1->fieldsEqual(*s2))
          return false;
      }

      return true;
    }


    bool
    Platform::sendMsg(Message& msg, std::string ip, int port)
    {
      msg.setTimeStamp();
      msg.setSource(TREX_ID);
      return m_adapter.send(&msg, ip, port);
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
      m_adapter.unbind();
      std::map<std::string, Announce *>::iterator it;
      m_receivedAnnounces.clear();
    }
  }
}

