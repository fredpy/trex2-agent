/*
 * ImcAdapter.cc
 *
 *  Created on: Apr 4, 2013
 *      Author: zp
 */

#include "trex/lsts/ImcAdapter.hh"

namespace TREX
{
  namespace LSTS
  {

    namespace {
      
      class graph_proxy :public ImcAdapter::tick_proxy {
      public:
        graph_proxy(graph const &ref):m_graph(ref) {}
        ~graph_proxy() {}
        
        tick_type current_tick() {
          return m_graph.getCurrentTick();
        }
        date_type tick_to_date(tick_type const &tck) {
          return m_graph.tickToTime(tck);
        }
        tick_type date_to_tick(date_type const &date) {
          return m_graph.timeToTick(date);
        }
        std::string date_str(tick_type const &tck) {
          return date_export(m_graph, tck);
        }
        std::string duration_str(tick_type const &tck) {
          return duration_export(m_graph, tck);
        }
        tick_type as_date(std::string const &date) {
          return m_graph.as_date(date);
        }
        tick_type as_duration(std::string const &date) {
          return m_graph.as_duration(date);
        }
        
      private:
        graph const &m_graph;
        
      };
      
    }
    
    
    ImcAdapter::ImcAdapter() :
        c_imc_header_length(sizeof(IMC::Header)),
        c_max_iridium_payload_length(260)
    {
      m_trex_id = 65000;
      m_platf_id = 0;
      m_iridium_req = 0;
      bfr = new uint8_t[65535];
      messenger = NULL;
    }

    Observation
    ImcAdapter::vehicleMediumObservation(VehicleMedium * msg)
    {

      if (msg != NULL)
      {
        switch (msg->medium)
        {
          case (VehicleMedium::VM_WATER):
            return Observation("medium", "Water");
            break;
          case (VehicleMedium::VM_UNDERWATER):
            return Observation("medium", "Underwater");
            break;
          case (VehicleMedium::VM_AIR):
            return Observation("medium", "Air");
            break;
          case (VehicleMedium::VM_GROUND):
            return Observation("medium", "Ground");
            break;
          default:
            break;
        }
      }
      return Observation("medium", "Unknown");
    }

    Observation
    ImcAdapter::estimatedStateObservation(EstimatedState * msg)
    {
      if (msg == NULL)
        return Observation("estate", "Boot");

      Observation obs("estate", "Position");

      m_platf_id = msg->getSource();

      double latitude, longitude;
      latitude = msg->lat;
      longitude = msg->lon;
      WGS84::displace(msg->x, msg->y, &latitude, &longitude);
      obs.restrictAttribute("latitude", FloatDomain(latitude));
      obs.restrictAttribute("longitude", FloatDomain(longitude));

      //msg->toText(std::cout);
      if (msg->depth > 0)
        obs.restrictAttribute("z", FloatDomain(msg->depth));
      else if (msg->alt > 0)
        obs.restrictAttribute("z", FloatDomain(-msg->alt));
      else if (msg->height != -1)
        obs.restrictAttribute("z", FloatDomain(msg->height + (-msg->z)));

      if (msg->depth > 0)
        obs.restrictAttribute("depth",
                              FloatDomain(msg->depth /*+ (- msg->z)*/));
      if (msg->alt > 0)
        obs.restrictAttribute("altitude", FloatDomain(msg->alt));
      if (msg->height != -1)
        obs.restrictAttribute("height", FloatDomain(msg->height + (-msg->z)));

      return obs;
    }

    void ImcAdapter::setReactorGraph(graph const &g)
    {
      m_cvt.reset(new graph_proxy(g));
    }

    Observation
    ImcAdapter::followRefStateObservation(FollowRefState * msg)
    {
      if (msg == NULL || msg->reference.isNull()
          || msg->control_src != m_trex_id
          || msg->state == FollowRefState::FR_TIMEOUT
          || msg->state == FollowRefState::FR_WAIT)
        return Observation("reference", "Boot");

      bool xy_near = (msg->proximity & FollowRefState::PROX_XY_NEAR) != 0;
      bool z_near = (msg->proximity & FollowRefState::PROX_Z_NEAR) != 0;

      Observation obs("refstate", "Going");

      obs.restrictAttribute("near_z", BooleanDomain((z_near)));
      obs.restrictAttribute("near_xy", BooleanDomain((xy_near)));

      obs.restrictAttribute("latitude", FloatDomain(msg->reference->lat));
      obs.restrictAttribute("longitude", FloatDomain(msg->reference->lon));

      if (!msg->reference->z.isNull())
      {
        switch (msg->reference->z->z_units)
        {
          case (Z_DEPTH):
            obs.restrictAttribute("z", FloatDomain(msg->reference->z->value));
            break;
          case (Z_ALTITUDE):
            obs.restrictAttribute("z", FloatDomain(-msg->reference->z->value));
            break;
          case (Z_HEIGHT):
            obs.restrictAttribute("z", FloatDomain(msg->reference->z->value));
            break;
          default:
            break;
        }
      }

      if (!msg->reference->speed.isNull())
      {
        obs.restrictAttribute("speed",
                              FloatDomain((msg->reference->speed->value)));
      }

      return obs;
    }

    Observation
    ImcAdapter::planControlStateObservation(PlanControlState * msg)
    {
      if (msg != NULL)
      {

        if (msg->state == PlanControlState::PCS_EXECUTING
            && msg->plan_id == "trex_plan")
        {
          Observation obs = Observation("control", "TREX");
          return obs;
        }
        return Observation("control", "DUNE");
      }

      return Observation("control", "Boot");
    }

    Observation
    ImcAdapter::opLimitsObservation(OperationalLimits * msg)
    {
      if (msg == NULL)
        return Observation("oplimits", "Boot");

      Observation obs("oplimits", "Limits");

      if (msg->mask & IMC::OPL_MAX_DEPTH)
        obs.restrictAttribute("max_depth", FloatDomain(msg->max_depth));

      if ((msg->mask & IMC::OPL_MAX_ALT))
        obs.restrictAttribute("max_altitude", FloatDomain(msg->max_altitude));

      if (msg->mask & IMC::OPL_MIN_ALT)
        obs.restrictAttribute("min_altitude", FloatDomain(msg->min_altitude));

      if (msg->mask & IMC::OPL_MAX_SPEED)
        obs.restrictAttribute("max_speed", FloatDomain(msg->max_speed));

      if (msg->mask & IMC::OPL_MIN_SPEED)
        obs.restrictAttribute("min_speed", FloatDomain(msg->min_speed));

      InsideOpLimits::set_oplimits(msg);

      return obs;
    }

    Observation
    ImcAdapter::announceObservation(Announce * msg)
    {
      std::string system = msg->sys_name;
      std::replace(system.begin(), system.end(), '-', '_');

      double age = Time::Clock::getSinceEpoch() - msg->getTimeStamp();

      if (age > 15)
      {
        Observation obs(system, "position");
        obs.restrictAttribute("latitude", FloatDomain(msg->lat));
        obs.restrictAttribute("longitude", FloatDomain(msg->lon));
        obs.restrictAttribute("height", FloatDomain(msg->height));
        return obs;
      }

      Observation obs(system, "connected");
      obs.restrictAttribute("latitude", FloatDomain(msg->lat));
      obs.restrictAttribute("longitude", FloatDomain(msg->lon));
      obs.restrictAttribute("height", FloatDomain(msg->height));

      return obs;
    }

    Goal
    ImcAdapter::genericGoal(TrexToken * msg, bool restrict_to_future)
    {
      Goal g(msg->timeline, msg->predicate);

      MessageList<TrexAttribute>::const_iterator it;
      for (it = msg->attributes.begin(); it != msg->attributes.end(); it++)
      {
        TrexAttribute * attr = *it;

        if (attr->name == "start" || attr->name == "end")
        {
          IntegerDomain::bound min = IntegerDomain::minus_inf,
            max = IntegerDomain::plus_inf;
          if( restrict_to_future )
            min = m_cvt->current_tick();
          if (!attr->min.empty())
            min = m_cvt->as_date(attr->min);
          if (!attr->max.empty())
            max = m_cvt->as_date(attr->max);

          if (attr->name == "start")
            g.restrictStart(IntegerDomain(min, max));
          else
            g.restrictEnd(IntegerDomain(min, max));
        }
        else if (attr->name == "duration")
        {
          IntegerDomain::bound min = 1, max = IntegerDomain::plus_inf;
          if (!attr->min.empty())
            min = m_cvt->as_duration(attr->min);
          if (!attr->max.empty())
            max = m_cvt->as_duration(attr->max);

          g.restrictDuration(IntegerDomain(min, max));
        }
        else
          setAttribute(g, *attr);
      }

      return g;
    }

    void
    ImcAdapter::setAttribute(Predicate &pred, TrexAttribute const &attr)
    {
      IntegerDomain::bound min_i, max_i;
      FloatDomain::bound min_f, max_f;
      //MessageList<TrexAttribute>::const_iterator it;
      std::string min = attr.min;
      std::string max = attr.max;

      switch (attr.attr_type)
      {
        case TrexAttribute::TYPE_STRING:
          pred.restrictAttribute(attr.name, StringDomain(min));
          break;

        case TrexAttribute::TYPE_BOOL:
          if (min == max && min != "")
            pred.restrictAttribute(attr.name,
                                   BooleanDomain(min != "false" || min != "0"));
          else
            pred.restrictAttribute(attr.name, BooleanDomain());
          break;

        case TrexAttribute::TYPE_INT:
          if (min == "")
            min_i = IntegerDomain::minus_inf;
          else
            min_i = strtoll(min.c_str(), NULL, 10);

          if (max == "")
            max_i = IntegerDomain::plus_inf;
          else
            max_i = strtoll(max.c_str(), NULL, 10);

          pred.restrictAttribute(attr.name, IntegerDomain(min_i, max_i));
          break;

        case TrexAttribute::TYPE_FLOAT:
          if (min == "")
            min_f = FloatDomain::minus_inf;
          else
            min_f = strtod(min.c_str(), NULL);

          if (max == "")
            max_f = FloatDomain::plus_inf;
          else
            max_f = strtod(max.c_str(), NULL);

          pred.restrictAttribute(attr.name, FloatDomain(min_f, max_f));
          break;

        case TrexAttribute::TYPE_ENUM:
          if (min == "" || max == "")
            pred.restrictAttribute(attr.name, EnumDomain());
          else
            pred.restrictAttribute(attr.name, EnumDomain(min));
          break;

        default:
          break;
      }
    }

    Observation
    ImcAdapter::genericObservation(TICK &date, TrexToken * msg)
    {
      Observation obs(msg->timeline, msg->predicate);

      MessageList<TrexAttribute>::const_iterator it;
      for (it = msg->attributes.begin(); it != msg->attributes.end(); it++)
      {
        setAttribute(obs, **it);
      }
      CHRONO::duration<double> t_stamp(msg->getTimeStamp());
      typedef chrono_posix_convert< CHRONO::duration<double> > cvt;
      graph::date_type
        pdate = boost::posix_time::from_time_t(0)+cvt::to_posix(t_stamp);
      
      date = m_cvt->date_to_tick(pdate);
      
      return obs;
    }

    bool
    ImcAdapter::sendAsynchronous(Message * msg, std::string addr, int port)
    {
       if (msg->getTimeStamp() <= 0)
         msg->setTimeStamp();

       std::cout << "TREX ID " << m_trex_id << std::endl;
       if (msg->getSource() == 0 || msg->getSource() == 65535)
         msg->setSource(m_trex_id);

       if (messenger == NULL)
        messenger = new ImcMessenger();
      messenger->post(msg, port, addr);
      return true;
    }

    bool
    ImcAdapter::sendViaIridium(Message * msg, const std::string address,
        int port)
    {
      if (msg->getTimeStamp() == 0)
    	  msg->setTimeStamp();

      uint8_t buffer[65635];
      ImcIridiumMessage * irMsg = new ImcIridiumMessage(msg);
      irMsg->destination = msg->getDestination();
      irMsg->source = m_platf_id;

      int len = irMsg->serialize(buffer);

      // message needs to be fragmented...
      if (len > c_max_iridium_payload_length)
      {
        int i;
        DUNE::Network::Fragments frags(msg, c_max_iridium_payload_length);
        std::cout << "Message to be sent via iridium fragmented into "
            << frags.getNumberOfFragments() << " fragments." << std::endl;

        for (i = 0; i < frags.getNumberOfFragments(); i++)
        {
          ImcIridiumMessage * irMsg = new ImcIridiumMessage(
              frags.getFragment(i));
          uint8_t buff[512];
          irMsg->destination = msg->getDestination();
          irMsg->source = m_platf_id;
          int length = irMsg->serialize(buff);
          IridiumMsgTx * tx = new IridiumMsgTx();
          tx->ttl = 1800; // try sending this update for 30 minutes
          tx->data.assign(buff, buff + length);
          tx->setTimeStamp();
          tx->req_id = (++m_iridium_req) % 65535;
          if (!send(tx, address, port))
            return false;
        }
        return true;
      }
      else
      {
        IridiumMsgTx * tx = new IridiumMsgTx();
        tx->setTimeStamp();
        tx->data.assign(buffer, buffer + len);
        tx->req_id = (++m_iridium_req) % 65535;
        return send(tx, address, port);
      }
    }

    Message *
    ImcAdapter::pollAsynchronous()
    {
      if (messenger == NULL)
        messenger = new ImcMessenger();
      return messenger->receive();
    }

    bool
    ImcAdapter::bindSynchronous(int port)
    {
      sock_receive.bind(port, Address::Any, true);
      m_poll.add(sock_receive);

      return true;
    }

    Message *
    ImcAdapter::pollSynchronous()
    {
      if (m_poll.poll(0))
      {
        Address addr;
        uint16_t rv = sock_receive.read(bfr, 65535, &addr);
        IMC::Message * msg = IMC::Packet::deserialize(bfr, rv);
        return msg;
      }
      return NULL;
    }

    bool
    ImcAdapter::sendSynchronous(Message * msg, std::string addr, int port)
    {
      if (msg->getTimeStamp() <= 0)
         msg->setTimeStamp();

      DUNE::Utils::ByteBuffer bb;
      try
      {
        IMC::Packet::serialize(msg, bb);

        return sock_send.write(bb.getBuffer(), msg->getSerializationSize(),
                               Address(addr.c_str()), port);
      }
      catch (std::runtime_error& e)
      {
        std::cerr << "ERROR: " << ": " << e.what() << "\n";
        return false;
      }
      return true;
    }

    bool
    ImcAdapter::bindAsynchronous(int port)
    {
      if (messenger != NULL)
        unbindAsynchronous();

      messenger = new ImcMessenger();
      messenger->startListening(port);

      return true;
    }

    bool
    ImcAdapter::unbindAsynchronous()
    {
      if (messenger != NULL)
      {
        messenger->stopListening();
        delete messenger;
      }
      messenger = NULL;
      return true;
    }

    bool
    ImcAdapter::unbindSynchronous()
    {
      m_poll.remove(sock_receive);

      //sock_receive.delFromPoll(iom);
      return true;
    }
//
//    bool
//    ImcAdapter::startDiscovery()
//    {
//      m_discovery.setMulticastTTL(1);
//      m_discovery.setMulticastLoop(false);
//      m_discovery.enableBroadcast(true);
//      std::vector<Interface> itfs = Interface::get();
//      int i;
//      for (unsigned i = 0; i < itfs.size(); ++i)
//        m_discovery.joinMulticastGroup("224.0.75.69", itfs[i].address());
//      for (i = 30100; i < 30105; i++)
//      {
//        try
//        {
//          m_discovery.bind(i, Address::Any, true);
//          std::cout << "listening for advertisements on port " << i << "\n";
//          m_discovery.addToPoll(m_diom);
//          return true;
//        }
//        catch (...)
//        { }
//      }
//      return false;
//    }

    void
    ImcAdapter::variableToImc(Variable const &v, TrexAttribute * attr)
    {
      Symbol t = v.domain().getTypeName();
      attr->name = v.name().str();

      if (attr->name == "start" || attr->name == "end")
      {
        attr->attr_type = TrexAttribute::TYPE_STRING;
        IntegerDomain const &id =
            dynamic_cast<IntegerDomain const &>(v.domain());
        if (id.isSingleton())
        {
          std::string s = m_cvt->date_str(id.lowerBound());
          attr->min = s;
          attr->max = s;
        }
        else
        {
          if (id.hasUpper())
          {
            std::string s = m_cvt->date_str(id.upperBound());
            attr->max = s;
          }
          if (id.hasLower())
          {
            std::string s = m_cvt->date_str(id.lowerBound());
            attr->min = s;
          }
        }
        return;
      }
      else if (attr->name == "duration")
      {
        attr->attr_type = TrexAttribute::TYPE_STRING;
        IntegerDomain const &id =
            dynamic_cast<IntegerDomain const &>(v.domain());
        if (id.isSingleton())
        {
          std::string s = m_cvt->duration_str(id.lowerBound());
          attr->min = s;
          attr->max = s;
        }
        else
        {
          if (id.hasUpper())
          {
            std::string s = m_cvt->duration_str(id.upperBound().value() + 1);
            attr->max = s;
          }
          if (id.hasLower())
          {
            std::string s = m_cvt->duration_str(id.lowerBound().value() - 1);
            attr->min = s;
          }
        }
        return;
      }
      else if (t.str() == "float")
      {
        attr->attr_type = TrexAttribute::TYPE_FLOAT;
      }
      else if (t.str() == "int")
      {
        attr->attr_type = TrexAttribute::TYPE_INT;
      }
      else if (t.str() == "bool")
      {
        attr->attr_type = TrexAttribute::TYPE_BOOL;
      }
      else if (t.str() == "string")
      {
        attr->attr_type = TrexAttribute::TYPE_STRING;
      }
      else if (t.str() == "enum")
      {
        attr->attr_type = TrexAttribute::TYPE_ENUM;
      }

      if (v.domain().isSingleton())
      {
        attr->max = v.domain().getStringSingleton();
        attr->min = v.domain().getStringSingleton();
      }
      else
      { // if (v.domain().isFull()) {
        attr->max = "";
        attr->min = "";
      }
    }

    void
    ImcAdapter::asImcMessage(TICK date, Predicate const &obs, TrexToken * result)
    {
      bool duration_full = true, end_deduced = true;
      
      result->timeline = obs.object().str();
      result->predicate = obs.predicate().str();
      result->attributes.clear();

      std::list<TREX::utils::Symbol> attrs;

      
      obs.listAttributes(attrs);
      std::list<TREX::utils::Symbol>::iterator it;
      
      if( obs.has_temporal_scope() ) {
        IntegerDomain duration(Goal::s_durationDomain);
        
        // check if duration is the default [1, +inf]
        duration.restrictWith(obs.getAttribute(Goal::s_durationName).domain());
        duration_full = duration.equals(Goal::s_durationDomain);
      
        // check if the end is directly deduced from start+duration
        IntegerDomain start(Goal::s_dateDomain);
          
        start.restrictWith(obs.getAttribute(Goal::s_startName).domain());
        Goal test("foo", "bar");
        test.restrictTime(start, duration, Goal::s_dateDomain);
        end_deduced = test.getEnd().equals(obs.getAttribute(Goal::s_endName).domain());
      }
      
      
      for (it = attrs.begin(); it != attrs.end(); it++)
      {
        TrexAttribute attr;
        Variable v = obs.getAttribute(*it);
        // test if the domain is not restricted
        bool full;
        // special case for duration which is [1, +inf]
        if( v.name()==Goal::s_durationName )
          full = duration_full;
        else if( v.name()==Goal::s_endName )
          full = end_deduced;
        else
          full = v.domain().isFull();
        
        // Do not add domains that are not restricted
        if( !full ) {
          variableToImc(v, &attr);
          result->attributes.push_back(attr);
        }
      }
      
      boost::posix_time::time_duration
        time = m_cvt->tick_to_date(date)-boost::posix_time::from_time_t(0);
      
      long double val = time.total_milliseconds();
      val /= 1000.0;
      result->setTimeStamp(static_cast<double>(val));
      result->setSource(m_trex_id);
    }

    void
    ImcAdapter::setTrexId(int trex_id)
    {
      m_trex_id = trex_id;
    }

    void
    ImcAdapter::setPlatformId(int platf_id)
    {
      m_platf_id = platf_id;
    }

    ImcAdapter::~ImcAdapter()
    {
      unbind();
    }
  }
}

