/*
 * YoYoReactor.cc
 *
 *  Created on: Jun 11, 2013
 *      Author: zp
 */
#include "trex/lsts/EuropaExtensions.hh"

#include "YoYoReactor.hh"

#include <trex/domain/BooleanDomain.hh>
#include <trex/domain/EnumDomain.hh>
#include <trex/domain/FloatDomain.hh>

namespace
{

  /** @brief TREX log entry point */
  SingletonUse<LogManager> s_log;

  /** @brief Platform reactor declaration */
  TeleoReactor::xml_factory::declare<TREX::LSTS::YoYoReactor> decl("YoYoReactor");

}

namespace TREX {
  namespace LSTS {

    // Symbol equality test is faster than string : use global Symbols to improve performances
    utils::Symbol const YoYoReactor::s_trex_pred("TREX");
    utils::Symbol const YoYoReactor::s_exec_pred("Exec");

    utils::Symbol const YoYoReactor::s_reference_tl("reference");
    utils::Symbol const YoYoReactor::s_refstate_tl("refstate");
    utils::Symbol const YoYoReactor::s_control_tl("control");
    utils::Symbol const YoYoReactor::s_position_tl("estate");
    utils::Symbol const YoYoReactor::s_yoyo_tl("yoyo");
    utils::Symbol const YoYoReactor::s_yoyo_state_tl("yoyo_state");

    YoYoReactor::YoYoReactor(TeleoReactor::xml_arg_type arg) :
              LstsReactor(arg),
              m_lastRefState(s_refstate_tl, "Failed"),
              m_lastControl(s_control_tl, "Failed"),
              m_lastReference(s_reference_tl, "Failed"),
              m_lastPosition(s_position_tl, "Failed")
    {

      m_pitch = Angles::radians(parse_attr<double>(15, TeleoReactor::xml_factory::node(arg),
                                                   "pitch"));

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
    YoYoReactor::handleInit()
    {
      Observation yoyo(s_yoyo_tl, "Idle");
      postObservation(yoyo);

      Observation yoyo_state(s_yoyo_state_tl, "Idle");
      postObservation(yoyo_state);

      m_lastSeenRef.lat = m_lastSeenRef.lon = m_lastSeenRef.speed =
          m_lastSeenRef.z = m_lastSeenRef.tick = 0;
      m_lastSentRef.lat = m_lastSentRef.lon = m_lastSentRef.speed =
          m_lastSentRef.z = m_lastSentRef.tick = 0;

    }

    void
    YoYoReactor::handleTickStart()
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
      bool nearXY = false, nearZ = false, nearBottom = false, nearEnd = false;
      double dist_to_target = 10000;

      Variable v;

      if (m_lastPosition.hasAttribute("altitude"))
      {
        v = m_lastPosition.getAttribute("altitude");
        FloatDomain const &alt = dynamic_cast<FloatDomain const &>(v.domain());

        double alt_ = v.domain().getTypedSingleton<double, true>();
        if (alt_ > 0 && alt_ < 4)
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

      if (m_lastPosition.hasAttribute("latitude") && m_lastPosition.hasAttribute("longitude") && m_lastPosition.hasAttribute("depth"))
      {
        Variable lat = m_lastPosition.getAttribute("latitude");
        Variable lon = m_lastPosition.getAttribute("longitude");
        Variable depth = m_lastPosition.getAttribute("depth");

        double cur_lat = lat.domain().getTypedSingleton<double,0>();
        double cur_lon = lon.domain().getTypedSingleton<double,0>();
        double cur_depth = depth.domain().getTypedSingleton<double,0>();

        dist_to_target = WGS84::distance(cur_lat, cur_lon, 0, m_lat, m_lon, 0);
        nearEnd = dist_to_target < (cur_depth / std::tan(m_pitch));
      }

      double atZ = -1000;
      if (m_lastPosition.hasAttribute("z"))
      {
        Variable vz = m_lastPosition.getAttribute("z");
        if (vz.isComplete() && vz.domain().isSingleton())
        {
          atZ = vz.domain().getTypedSingleton<double,true>();
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
      //std::cerr << "nearZ: " << nearZ<< ", atZ: "<< atZ << ", m_cmdz: "<< m_cmdz<< std::endl;

      if (m_lastRefState.hasAttribute("near_xy"))
      {
        v = m_lastRefState.getAttribute("near_xy");

        BooleanDomain const &near_XY = dynamic_cast<BooleanDomain const &>(v.domain());

        if (near_XY.isSingleton())
        {
          nearXY = near_XY.getStringSingleton() == "true" || near_XY.getStringSingleton() == "1";
        }
      }

      if( is_verbose() )
	syslog(log::info) << "nearZ: " << nearZ<< ", atZ: "<< atZ << ", nearXY: " << nearXY << ", nearEnd: " <<
          nearEnd << ", nearBottom: "<<nearBottom << std::endl;

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
        else if (nearZ || nearBottom || nearEnd)
        {
          syslog(log::info)<< "Now going up...";
          requestReference(m_lat, m_lon, m_speed, m_minz);
          state = ASCEND;
        }
        break;

        case (SURFACE):
                if (m_lastReference.predicate() == "At" && nearXY && nearZ) // arrived at destination
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
	  if( is_verbose() )
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

      g.restrictAttribute(Variable("latitude", FloatDomain(lat)));
      g.restrictAttribute(Variable("longitude", FloatDomain(lon)));
      g.restrictAttribute(Variable("z", FloatDomain(z)));
      g.restrictAttribute(Variable("speed", FloatDomain(speed)));

      postGoal(g);

      m_lastSentRef.lat = lat;
      m_lastSentRef.lon = lon;
      m_lastSentRef.z = z;
      m_lastSentRef.speed = speed;
      m_lastSentRef.tick = getCurrentTick();
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
        else
        {
        	syslog(log::warn)<< "won't handle this request because latitude is not singleton!";
        	return;
        }

        v = g->getAttribute("longitude");
        if (v.domain().isSingleton())
          m_lon = v.domain().getTypedSingleton<double, true>();
        else
        {
        	syslog(log::warn)<< "won't handle this request because longitude is not singleton!";
        	return;
        }
	DUNE::IMC::OperationalLimits const * lim = InsideOpLimits::get_oplimits();
	if( NULL!=lim && ( lim->mask & DUNE::IMC::OPL_AREA ) ) {
	  double x,y;
	  WGS84::displacement(lim->lat, lim->lon, 0, m_lat, m_lon, 0, &x, &y);
	  Angles::rotate(lim->orientation, true, x, y);

	  double d2limits = std::max(std::fabs(x) - 0.5 * lim->length,
				     std::fabs(y) - 0.5 * lim->width);
	  if( 0 <= d2limits ) {
	    std::ostringstream oss;
	    
	    oss<< "Ignoring invalid Yo Yo: "<<*g
	       <<"\n   reason: ("<<DUNE::Math::Angles::degrees(m_lat)
	       <<','<<DUNE::Math::Angles::degrees(m_lon)
	       <<") is outside of safety limits)";
	    syslog(log::error)<<oss.str();
	    std::cerr<<oss.str()<<std::endl;
	    
	    return;
	  }
	} 

	

        v = g->getAttribute("speed");
        if (v.domain().isSingleton())
          m_speed = v.domain().getTypedSingleton<double, true>();

        v = g->getAttribute("min_z");
        if (v.domain().isSingleton())
          m_minz = v.domain().getTypedSingleton<double, true>();

        v = g->getAttribute("max_z");
        if (v.domain().isSingleton())
          m_maxz = v.domain().getTypedSingleton<double, true>();

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
    YoYoReactor::handleRecall(TREX::transaction::goal_id const &g)
    {
      //std::cerr << "[YOYO] handleRecall(" << (*g) << ")" << std::endl;
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
          m_lastSeenRef.tick = getCurrentTick();
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
