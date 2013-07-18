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
  SingletonUse<LogManager> s_log;

  /** @brief Platform reactor declaration */
  TeleoReactor::xml_factory::declare<TREX::LSTS::YoYoReactor> decl("YoYoReactor");

}

namespace TREX {
  namespace LSTS {

    // Sy,bol equaity test is faster than string : use global Symbols to improve performances
    utils::Symbol const YoYoReactor::s_trex_pred("TREX");
    utils::Symbol const YoYoReactor::s_exec_pred("Exec");

    utils::Symbol const YoYoReactor::s_reference_tl("reference");
    utils::Symbol const YoYoReactor::s_refstate_tl("refstate");
    utils::Symbol const YoYoReactor::s_control_tl("control");
    utils::Symbol const YoYoReactor::s_position_tl("estate");

    utils::Symbol const YoYoReactor::s_yoyo_tl("yoyo");


    YoYoReactor::YoYoReactor(TeleoReactor::xml_arg_type arg) :
                    LstsReactor(arg),
                    m_lastRefState(s_refstate_tl, "Failed"),
                    m_lastControl(s_control_tl, "Failed"),
                    m_lastReference(s_reference_tl, "Failed"),
                    m_lastPosition(s_position_tl, "Failed")
    {
      m_lat = m_lon = m_speed = m_minz = m_maxz = -1;
      //      m_time_underwater = 0;
      m_time_at_surface = 0;

      //      m_secs_underwater = 0;
      state = IDLE;
      use(s_reference_tl, true);
      use(s_refstate_tl, false);
      use(s_control_tl);
      use(s_position_tl);
      provide(s_yoyo_tl);
    }

    void
    YoYoReactor::handleInit()
    {
      Observation yoyo(s_yoyo_tl, "Idle");
      postObservation(yoyo);
    }

    void
    YoYoReactor::handleTickStart()
    {
      //std::cerr << "[YOYO] handleTickStart()" << std::endl;
    }

    void
    YoYoReactor::printReference(ReferenceRequest req) {
      syslog(log::warn) << "Reference ( lat: " << req.lat << ", lon: " << req.lon << " / z: " << req.z << ", speed: " << req.speed << ")";
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
      Variable v;
      int secs_at_surface = 60;

     /* if (m_lastPosition.hasAttribute("altitude"))
      {
        v = m_lastPosition.getAttribute("altitude");
        FloatDomain const &alt = dynamic_cast<FloatDomain const &>(v.domain());

        double alt_ = v.domain().getTypedSingleton<double, true>();
        if (alt_ > 0 && alt_ < 2)
        {
          nearBottom = true;
          syslog(log::warn) << "Close to bottom: " << alt;
        }
      }*/

      if (!sameReference(m_lastSeenRef, m_lastSentRef)) {
	std::cerr << "the seen reference doesn't match my command. nothing to do." << std::endl;
        return true;
      }


      double atZ = -1000;
      if (m_lastPosition.hasAttribute("z"))
      {
        Variable vz = m_lastPosition.getAttribute("z");
        if (vz.isComplete() && vz.domain().isSingleton())
        {
	   atZ = vz.domain().getTypedSingleton<double,true>();      
	   std::cerr << "VZ IS NOW " << vz<< ", atZ: " << atZ << std::endl;     
        }
      }

      if (m_lastRefState.hasAttribute("near_z"))
      {

        v = m_lastRefState.getAttribute("near_z");
        BooleanDomain const &nearz = dynamic_cast<BooleanDomain const &>(v.domain());

        if (nearz.isSingleton())
        {
          nearZ = nearz.getStringSingleton() == "true" || nearz.getStringSingleton() == "1";
        }
      }

      nearZ = nearZ && abs(atZ - m_cmdz) < 2;
      std::cerr << "nearZ: " << nearZ<< ", atZ: "<< atZ << ", m_cmdz: "<< m_cmdz<< std::endl;

      if (m_lastRefState.hasAttribute("near_xy"))
      {
        v = m_lastRefState.getAttribute("near_xy");

        BooleanDomain const &near_XY = dynamic_cast<BooleanDomain const &>(v.domain());

        if (near_XY.isSingleton())
        {
          nearXY = near_XY.getStringSingleton() == "true" || near_XY.getStringSingleton() == "1";
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
        if (nearXY && nearZ)
        {
          syslog(log::info)<< "Finished executing yoyo...";
          Observation obs = Observation(s_yoyo_tl, "Done");
          obs.restrictAttribute("latitude", FloatDomain(m_lat, m_lat));
          obs.restrictAttribute("longitude", FloatDomain(m_lon, m_lon));
          obs.restrictAttribute("speed", FloatDomain(m_speed, m_speed));
          obs.restrictAttribute("max_z", FloatDomain(m_maxz, m_maxz));
          obs.restrictAttribute("min_z", FloatDomain(m_minz, m_minz));
          postUniqueObservation(obs);
          state = IDLE;
        }
        break;
        default:
          postUniqueObservation(Observation(s_yoyo_tl, "Idle"));
          break;
      }

      return true;
    }

    void
    YoYoReactor::requestReference(double lat, double lon, double speed, double z)
    {
      Goal g(s_reference_tl, "Going");

      g.restrictAttribute(Variable("latitude", FloatDomain(lat)));
      g.restrictAttribute(Variable("longitude", FloatDomain(lon)));
      g.restrictAttribute(Variable("z", FloatDomain(z)));
      g.restrictAttribute(Variable("speed", FloatDomain(speed)));

      //std::cerr << "[YOYO] Sent reference request (" << lat << ", " << lon << ", " << speed << ", " << z << ")" << std::endl;

      postGoal(g);

      m_lastSentRef.lat = lat;
      m_lastSentRef.lon = lon;
      m_lastSentRef.z = z;
      m_lastSentRef.speed = speed;
      m_cmdz = z;
    }

    void
    YoYoReactor::handleRequest(TREX::transaction::goal_id const &goal)
    {
      if ( s_trex_pred != m_lastControl.predicate() )
      {
        syslog(log::warn)<< "won't handle this request because TREX is not controlling the vehicle!";
        return;
      }

      // Make a local copy to increase my reference counter instead of accessing the raw pointer directly !!!!!
      goal_id g = goal;
      Variable v;

      if ( g->predicate() == s_exec_pred )
      {
        v = g->getAttribute("latitude");
        if (v.domain().isSingleton())
          m_lat = v.domain().getTypedSingleton<double, true>();

        v = g->getAttribute("longitude");
        if (v.domain().isSingleton())
          m_lon = v.domain().getTypedSingleton<double, true>();

        v = g->getAttribute("speed");
        if (v.domain().isSingleton())
          m_speed = v.domain().getTypedSingleton<double, true>();

        v = g->getAttribute("min_z");
        if (v.domain().isSingleton())
          m_minz = v.domain().getTypedSingleton<double, true>();

        v = g->getAttribute("max_z");
        if (v.domain().isSingleton())
          m_maxz = v.domain().getTypedSingleton<double, true>();

        state = DESCEND;
        requestReference(m_lat, m_lon, m_speed, m_maxz);
        postUniqueObservation(*g);
      }
      else
      {
        syslog(log::warn)<< "Request is not valid: " << g->predicate();
      }
    }

    void
    YoYoReactor::handleRecall(TREX::transaction::goal_id const &g)
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
          m_lastSeenRef.lat = obs.getAttribute("latitude").domain().getTypedSingleton<double,true>();
          m_lastSeenRef.lon = obs.getAttribute("longitude").domain().getTypedSingleton<double,true>();
          m_lastSeenRef.speed = obs.getAttribute("speed").domain().getTypedSingleton<double,true>();
          m_lastSeenRef.z = obs.getAttribute("z").domain().getTypedSingleton<double,true>();
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
