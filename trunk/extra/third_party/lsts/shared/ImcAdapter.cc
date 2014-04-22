/*
 * ImcAdapter.cc
 *
 *  Created on: Apr 4, 2013
 *      Author: zp
 */

#include "ImcAdapter.hh"

namespace TREX
{
  namespace LSTS
  {

    ImcAdapter::ImcAdapter() :
        c_imc_header_length(sizeof(IMC::Header)),
        c_max_iridium_payload_length(260)
    {
      m_trex_id = 65000;
      m_graph = NULL;
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

      double latitude, longitude;
      latitude = msg->lat;
      longitude = msg->lon;
      WGS84::displace(msg->x, msg->y, &latitude, &longitude);
      obs.restrictAttribute("latitude", float_domain(latitude));
      obs.restrictAttribute("longitude", float_domain(longitude));

      //msg->toText(std::cout);
      if (msg->depth > 0)
        obs.restrictAttribute("z", float_domain(msg->depth));
      else if (msg->alt > 0)
        obs.restrictAttribute("z", float_domain(-msg->alt));
      else if (msg->height != -1)
        obs.restrictAttribute("z", float_domain(msg->height + (-msg->z)));

      if (msg->depth > 0)
        obs.restrictAttribute("depth",
                              float_domain(msg->depth /*+ (- msg->z)*/));
      if (msg->alt > 0)
        obs.restrictAttribute("altitude", float_domain(msg->alt));
      if (msg->height != -1)
        obs.restrictAttribute("height", float_domain(msg->height + (-msg->z)));

      return obs;
    }

    void
    ImcAdapter::setReactorGraph(graph const &g)
    {
      m_graph = &g;
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

      obs.restrictAttribute("near_z", boolean_domain((z_near)));
      obs.restrictAttribute("near_xy", boolean_domain((xy_near)));

      obs.restrictAttribute("latitude", float_domain(msg->reference->lat));
      obs.restrictAttribute("longitude", float_domain(msg->reference->lon));

      if (!msg->reference->z.isNull())
      {
        switch (msg->reference->z->z_units)
        {
          case (Z_DEPTH):
            obs.restrictAttribute("z", float_domain(msg->reference->z->value));
            break;
          case (Z_ALTITUDE):
            obs.restrictAttribute("z", float_domain(-msg->reference->z->value));
            break;
          case (Z_HEIGHT):
            obs.restrictAttribute("z", float_domain(msg->reference->z->value));
            break;
          default:
            break;
        }
      }

      if (!msg->reference->speed.isNull())
      {
        obs.restrictAttribute("speed",
                              float_domain((msg->reference->speed->value)));
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
        obs.restrictAttribute("max_depth", float_domain(msg->max_depth));

      if ((msg->mask & IMC::OPL_MAX_ALT))
        obs.restrictAttribute("max_altitude", float_domain(msg->max_altitude));

      if (msg->mask & IMC::OPL_MIN_ALT)
        obs.restrictAttribute("min_altitude", float_domain(msg->min_altitude));

      if (msg->mask & IMC::OPL_MAX_SPEED)
        obs.restrictAttribute("max_speed", float_domain(msg->max_speed));

      if (msg->mask & IMC::OPL_MIN_SPEED)
        obs.restrictAttribute("min_speed", float_domain(msg->min_speed));

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
        obs.restrictAttribute("latitude", float_domain(msg->lat));
        obs.restrictAttribute("longitude", float_domain(msg->lon));
        obs.restrictAttribute("height", float_domain(msg->height));
        return obs;
      }

      Observation obs(system, "connected");
      obs.restrictAttribute("latitude", float_domain(msg->lat));
      obs.restrictAttribute("longitude", float_domain(msg->lon));
      obs.restrictAttribute("height", float_domain(msg->height));

      return obs;
    }

    Goal
    ImcAdapter::genericGoal(TrexToken * msg)
    {
      Goal g(msg->timeline, msg->predicate);

      MessageList<TrexAttribute>::const_iterator it;
      for (it = msg->attributes.begin(); it != msg->attributes.end(); it++)
      {
        TrexAttribute * attr = *it;

        if (attr->name == "start" || attr->name == "end")
        {
          int_domain::bound
            min_v = m_graph->getCurrentTick(), max_v = int_domain::plus_inf;
          if (!attr->min.empty())
            min_v = m_graph->as_date(attr->min);
          if (!attr->max.empty())
            max_v = m_graph->as_date(attr->max);

          if (attr->name == "start")
            g.restrictStart(int_domain(min_v, max_v));
          else
            g.restrictEnd(int_domain(min_v, max_v));
        }
        else if (attr->name == "duration")
        {
          int_domain::bound min_v = 1, max_v = int_domain::plus_inf;
          if (!attr->min.empty())
            min_v = m_graph->as_duration(attr->min);
          if (!attr->max.empty())
            max_v = m_graph->as_duration(attr->max);

          g.restrictDuration(int_domain(min_v, max_v));
        }
        else
          setAttribute(g, *attr);
      }

      return g;
    }

    void
    ImcAdapter::setAttribute(Predicate &pred, TrexAttribute const &attr)
    {
      int_domain::bound min_i, max_i;
      float_domain::bound min_f, max_f;
      //MessageList<TrexAttribute>::const_iterator it;
      std::string min_v = attr.min;
      std::string max_v = attr.max;

      switch (attr.attr_type)
      {
        case TrexAttribute::TYPE_STRING:
          pred.restrictAttribute(attr.name, string_domain(min_v));
          break;

        case TrexAttribute::TYPE_BOOL:
          if (min_v == max_v && min_v != "")
            pred.restrictAttribute(attr.name,
                                   boolean_domain(min_v != "false" || min_v != "0"));
          else
            pred.restrictAttribute(attr.name, boolean_domain());
          break;

        case TrexAttribute::TYPE_INT:
          if (min_v == "")
            min_i = int_domain::minus_inf;
          else
            min_i = strtoll(min_v.c_str(), NULL, 10);

          if (max_v == "")
            max_i = int_domain::plus_inf;
          else
            max_i = strtoll(max_v.c_str(), NULL, 10);

          pred.restrictAttribute(attr.name, int_domain(min_i, max_i));
          break;

        case TrexAttribute::TYPE_FLOAT:
          if (min_v == "")
            min_f = float_domain::minus_inf;
          else
            min_f = strtod(min_v.c_str(), NULL);

          if (max_v == "")
            max_f = float_domain::plus_inf;
          else
            max_f = strtod(max_v.c_str(), NULL);

          pred.restrictAttribute(attr.name, float_domain(min_f, max_f));
          break;

        case TrexAttribute::TYPE_ENUM:
          if (min_v == "" || max_v == "")
            pred.restrictAttribute(attr.name, enum_domain());
          else
            pred.restrictAttribute(attr.name, enum_domain(min_v));
          break;

        default:
          break;
      }
    }

    Observation
    ImcAdapter::genericObservation(TrexToken * msg)
    {
      Observation obs(msg->timeline, msg->predicate);

      MessageList<TrexAttribute>::const_iterator it;
      for (it = msg->attributes.begin(); it != msg->attributes.end(); it++)
      {
        setAttribute(obs, **it);
      }

      return obs;
    }

    bool
    ImcAdapter::sendAsynchronous(Message * msg, std::string addr, int port)
    {
      if (messenger == NULL)
        messenger = new ImcMessenger();
      messenger->post(msg, port, addr);
      return true;
    }

    bool
    ImcAdapter::sendViaIridium(Message * msg, const std::string address,
        int port)
    {
      uint8_t buffer[65635];
      ImcIridiumMessage * irMsg = new ImcIridiumMessage(msg);
      irMsg->destination = msg->getDestination();
      irMsg->source = msg->getSource();
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
          irMsg->source = msg->getSource();
          int length = irMsg->serialize(buff);
          IridiumMsgTx * tx = new IridiumMsgTx();
          tx->ttl = 1800; // try sending this update for 30 minutes
          tx->data.assign(buff, buff + length);
          if (!send(tx, address, port))
            return false;
        }
        return true;
      }
      else
      {
        IridiumMsgTx * tx = new IridiumMsgTx();
        tx->data.assign(buffer, buffer + len);
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
    ImcAdapter::variableToImc(var const &v, TrexAttribute * attr)
    {
      symbol t = v.domain().type_name();
      attr->name = v.name().str();

      if (attr->name == "start" || attr->name == "end")
      {
        attr->attr_type = TrexAttribute::TYPE_STRING;
        int_domain const &id =
            dynamic_cast<int_domain const &>(v.domain());
        if (id.is_singleton())
        {
          std::string s = date_export(*m_graph, id.lower_bound().value());
          attr->min = s;
          attr->max = s;
        }
        else
        {
          if (id.has_upper())
          {
            std::string s = date_export(*m_graph, id.upper_bound().value());
            attr->max = s;
          }
          if (id.has_lower())
          {
            std::string s = date_export(*m_graph, id.lower_bound().value());
            attr->min = s;
          }
        }
        return;
      }
      else if (attr->name == "duration")
      {
        attr->attr_type = TrexAttribute::TYPE_STRING;
        int_domain const &id =
            dynamic_cast<int_domain const &>(v.domain());
        if (id.is_singleton())
        {
          std::string s = duration_export(*m_graph, id.lower_bound().value());
          attr->min = s;
          attr->max = s;
        }
        else
        {
          if (id.has_upper())
          {
            std::string s = duration_export(*m_graph,
                                            id.upper_bound().value() + 1);
            attr->max = s;
          }
          if (id.has_lower())
          {
            std::string s = duration_export(*m_graph,
                                            id.lower_bound().value() - 1);
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

      if (v.domain().is_singleton())
      {
        attr->max = v.domain().get_singleton_as_string();
        attr->min = v.domain().get_singleton_as_string();
      }
      else
      { // if (v.domain().isFull()) {
        attr->max = "";
        attr->min = "";
      }
    }

    void
    ImcAdapter::asImcMessage(Predicate const &obs, TrexToken * result)
    {
      result->timeline = obs.object().str();
      result->predicate = obs.predicate().str();
      result->attributes.clear();

      std::list<TREX::utils::symbol> attrs;

      obs.listAttributes(attrs);
      std::list<TREX::utils::symbol>::iterator it;
      for (it = attrs.begin(); it != attrs.end(); it++)
      {
        TrexAttribute attr;
        var v = obs.getAttribute(*it);
        variableToImc(v, &attr);
        result->attributes.push_back(attr);
      }
    }

    void
    ImcAdapter::setTrexId(int trex_id)
    {
      m_trex_id = trex_id;
    }

    ImcAdapter::~ImcAdapter()
    {
      unbind();
    }
  }
}

