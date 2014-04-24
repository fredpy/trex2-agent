/*
 * YoYoReactor.cc
 *
 *  Created on: Jun 11, 2013
 *      Author: zp
 */

#include "YoYoReactor.hh"

namespace
{

  /** @brief TREX log entry point */
  singleton::use<log_manager> s_log;

  /** @brief Platform reactor declaration */
  reactor::factory::declare<TREX::LSTS::YoYoReactor> decl("YoYoReactor");

}

namespace TREX {
  namespace LSTS {

    // Symbol equality test is faster than string : use global Symbols to improve performances
    utils::symbol const YoYoReactor::s_trex_pred("TREX");
    utils::symbol const YoYoReactor::s_exec_pred("Exec");

    utils::symbol const YoYoReactor::s_reference_tl("reference");
    utils::symbol const YoYoReactor::s_refstate_tl("refstate");
    utils::symbol const YoYoReactor::s_control_tl("control");
    utils::symbol const YoYoReactor::s_position_tl("estate");
    utils::symbol const YoYoReactor::s_yoyo_tl("yoyo");
    utils::symbol const YoYoReactor::s_yoyo_state_tl("yoyo_state");

    YoYoReactor::YoYoReactor(reactor::xml_arg_type arg) :
      LstsReactor(arg),
      m_lastRefState(s_refstate_tl, "Failed"),
      m_lastControl(s_control_tl, "Failed"),
      m_lastReference(s_reference_tl, "Failed"),
      m_lastPosition(s_position_tl, "Failed")
    {
      m_lat = m_lon = m_speed = m_minz = m_maxz = -1;
      m_time_at_surface = 0;
      m_cmdz = 0;

      state = IDLE;
      use(s_reference_tl, true);
      use(s_refstate_tl, false);
      use(s_control_tl);
      use(s_position_tl);
      provide(s_yoyo_tl);
      provide(s_yoyo_state_tl);
    }

    void
    YoYoReactor::handle_init()
    {
      Observation yoyo(s_yoyo_tl, "Idle");
      post_observation(yoyo);

      Observation yoyo_state(s_yoyo_state_tl, "Idle");
      post_observation(yoyo_state);

      m_lastSeenRef.lat = m_lastSeenRef.lon = m_lastSeenRef.speed =
          m_lastSeenRef.z = m_lastSeenRef.tick = 0;
      m_lastSentRef.lat = m_lastSentRef.lon = m_lastSentRef.speed =
          m_lastSentRef.z = m_lastSentRef.tick = 0;

    }

    void
    YoYoReactor::handle_tick_start()
    {
    }

    void
    YoYoReactor::printReference(ReferenceRequest req) {
      std::cerr << "Reference ( lat: " << req.lat << ", lon: " << req.lon << " / z: " << req.z << ", speed: " << req.speed << ")\n";
    }

    bool
    sameReference(ReferenceRequest req1, ReferenceRequest req2)
    {
      if (req1.lat != req2.lat)
        return false;
      if (req1.lon != req2.lon)
        return false;
      if (req1.z != req2.z)
        return false;
      if (req1.speed != req2.speed)
        return false;
      return true;
    }

    bool
    YoYoReactor::synchronize()
    {
      bool nearXY = false, nearZ = false, nearBottom = false;
      var v;
      // int secs_at_surface = 60; //< unused variable

      if (m_lastPosition.hasAttribute("altitude"))
      {
        v = m_lastPosition.getAttribute("altitude");
        float_domain const &alt = dynamic_cast<float_domain const &>(v.domain());

        double alt_ = v.domain().get_typed_singleton<double, true>();
        if (alt_ > 0 && alt_ < 2)
        {
          nearBottom = true;
          syslog(log::warn) << "Close to bottom: " << alt;
        }
      }

      // If reference not yet sent or if other system is controlling the vehicle
      if (!sameReference(m_lastSeenRef, m_lastSentRef)) {
        std::cerr << "Seen reference doesn't match sent command." << std::endl;
        printReference(m_lastSeenRef);
        printReference(m_lastSentRef);
        return true;
      }
      /*else {
        std::cerr << "Requested at " << m_lastSentRef.tick << " and started at " << m_lastSeenRef.tick << std::endl;
      }*/

      double atZ = -1000;
      if (m_lastPosition.hasAttribute("z"))
      {
        var vz = m_lastPosition.getAttribute("z");
        if (vz.is_complete() && vz.domain().is_singleton())
        {
          atZ = vz.domain().get_typed_singleton<double,true>();
        }
      }

      if (m_lastRefState.hasAttribute("near_z"))
      {
        v = m_lastRefState.getAttribute("near_z");
        boolean_domain const &nearz = dynamic_cast<boolean_domain const &>(v.domain());

        if (nearz.is_singleton())
        {
          nearZ = nearz.get_singleton_as_string() == "true" || nearz.get_singleton_as_string() == "1";
        }
      }

      nearZ = nearZ && abs(atZ - m_cmdz) < 2;
      //std::cerr << "nearZ: " << nearZ<< ", atZ: "<< atZ << ", m_cmdz: "<< m_cmdz<< std::endl;

      if (m_lastRefState.hasAttribute("near_xy"))
      {
        v = m_lastRefState.getAttribute("near_xy");

        boolean_domain const &near_XY = dynamic_cast<boolean_domain const &>(v.domain());

        if (near_XY.is_singleton())
        {
          nearXY = near_XY.get_singleton_as_string() == "true" || near_XY.get_singleton_as_string() == "1";
        }
      }

      switch(state)
      {
        case (ASCEND):
            m_time_at_surface  = 0;

        if (nearXY)
        {
          syslog(log::warn)<< "Arrived. now surfacing...";
          requestReference(m_lat, m_lon, m_speed, 0);
          state = SURFACE;
        }
        else if (nearZ)
        {
          syslog(log::info)<< "Arrived at min depth, now going down...";
          requestReference(m_lat, m_lon, m_speed, m_maxz);
          state = DESCEND;
        }
        break;

        case (DESCEND):
        m_time_at_surface  = 0;
        if (nearXY)
        {
          syslog(log::info)<< "Arrived. now surfacing...";
          requestReference(m_lat, m_lon, m_speed, 0);
          state = SURFACE;
        }
        else if (nearZ || nearBottom)
        {
          syslog(log::info)<< "Arrived at max depth, now going up...";
          requestReference(m_lat, m_lon, m_speed, m_minz);
          state = ASCEND;
        }
        break;

        case (SURFACE):

        if (m_lastReference.predicate() == "At" && nearXY && nearZ) // arrived at destination
        {
          syslog(log::info)<< "Finished executing yoyo...";
          Observation obs = Observation(s_yoyo_tl, "Done");
          obs.restrictAttribute("latitude", float_domain(m_lat));
          obs.restrictAttribute("longitude", float_domain(m_lon));
          obs.restrictAttribute("speed", float_domain(m_speed));
          obs.restrictAttribute("max_z", float_domain(m_maxz));
          obs.restrictAttribute("min_z", float_domain(m_minz));
          postUniqueObservation(obs);
          state = IDLE;
        }
        break;
        default:
          syslog(log::info)<< "Just idling...";
          postUniqueObservation(Observation(s_yoyo_tl, "Idle"));
          break;
      }

      switch (state)
      {
        case (DESCEND):
          postUniqueObservation(Observation(s_yoyo_state_tl, "Descending"));
          break;
        case (ASCEND):
          postUniqueObservation(Observation(s_yoyo_state_tl, "Ascending"));
          break;
        case (SURFACE):
          postUniqueObservation(Observation(s_yoyo_state_tl, "Surfacing"));
          break;
        case (IDLE):
          postUniqueObservation(Observation(s_yoyo_state_tl, "Idle"));
          break;
      }

      return true;
    }

    void
    YoYoReactor::requestReference(double lat, double lon, double speed, double z)
    {
      Goal g(s_reference_tl, "Going");

      g.restrictAttribute(var("latitude", float_domain(lat)));
      g.restrictAttribute(var("longitude", float_domain(lon)));
      g.restrictAttribute(var("z", float_domain(z)));
      g.restrictAttribute(var("speed", float_domain(speed)));

      //std::cerr << "[YOYO] Sent reference request (" << lat << ", " << lon << ", " << speed << ", " << z << ")" << std::endl;

      post_goal(g);

      m_lastSentRef.lat = lat;
      m_lastSentRef.lon = lon;
      m_lastSentRef.z = z;
      m_lastSentRef.speed = speed;
      m_lastSentRef.tick = current_tick();
      m_cmdz = z;
    }

    void
    YoYoReactor::handle_request(TREX::transaction::goal_id const &goal)
    {
      if ( s_trex_pred != m_lastControl.predicate() )
      {
        syslog(log::warn)<< "won't handle this request because TREX is not controlling the vehicle!";
        return;
      }

      // Make a local copy to increase my reference counter instead of accessing the raw pointer directly !!!!!
      goal_id g = goal;
      var v;

      if ( g->predicate() == s_exec_pred )
      {
        v = g->getAttribute("latitude");
        if (v.domain().is_singleton())
          m_lat = v.domain().get_typed_singleton<double, true>();

        v = g->getAttribute("longitude");
        if (v.domain().is_singleton())
          m_lon = v.domain().get_typed_singleton<double, true>();

        v = g->getAttribute("speed");
        if (v.domain().is_singleton())
          m_speed = v.domain().get_typed_singleton<double, true>();

        v = g->getAttribute("min_z");
        if (v.domain().is_singleton())
          m_minz = v.domain().get_typed_singleton<double, true>();

        v = g->getAttribute("max_z");
        if (v.domain().is_singleton())
          m_maxz = v.domain().get_typed_singleton<double, true>();

        requestReference(m_lat, m_lon, m_speed, m_maxz);
        state = DESCEND;
        postUniqueObservation(*g);
      }
      else
      {
        syslog(log::warn)<< "Request is not valid: " << g->predicate();
      }
    }

    void
    YoYoReactor::handle_recall(TREX::transaction::goal_id const &g)
    {
      //std::cerr << "[YOYO] handleRecall(" << *(g.get()) << ")" << std::endl;
    }

    void
    YoYoReactor::notify(TREX::transaction::Observation const &obs)
    {
      if (s_reference_tl == obs.object())
      {
        m_lastReference = obs;
        if (m_lastReference.predicate() == "Going")
        {
          m_lastSeenRef.lat = obs.getAttribute("latitude").domain().get_typed_singleton<double,true>();
          m_lastSeenRef.lon = obs.getAttribute("longitude").domain().get_typed_singleton<double,true>();
          m_lastSeenRef.speed = obs.getAttribute("speed").domain().get_typed_singleton<double,true>();
          m_lastSeenRef.z = obs.getAttribute("z").domain().get_typed_singleton<double,true>();
          m_lastSeenRef.tick = current_tick();
        }
        else {

        }
      }
      else if (s_refstate_tl == obs.object())
        m_lastRefState = obs;
      else if (s_control_tl == obs.object())
      {
        m_lastControl = obs;
        if (m_lastControl.predicate() != "TREX")
        {
          state = IDLE;
        }
      }
      else if (s_position_tl == obs.object())
        m_lastPosition = obs;
    }

    YoYoReactor::~YoYoReactor()
    {
      // TODO Auto-generated destructor stub
    }
  }
}
