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

    utils::Symbol const YoYoReactor::s_yoyo_tl("yoyo");


    YoYoReactor::YoYoReactor(TeleoReactor::xml_arg_type arg) :
                LstsReactor(arg),
                m_lastRefState(s_refstate_tl, "Failed"),
                m_lastControl(s_control_tl, "Failed"),
                m_lastReference(s_reference_tl, "Failed")
    {
      m_lat = m_lon = m_speed = m_minz = m_maxz = -1;
      m_time_underwater = 0;
      m_time_at_surface = 0;
      m_secs_underwater = 0;
      state = IDLE;
      use(s_reference_tl, true);
      use(s_refstate_tl, false);
      use(s_control_tl);
      
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

    bool
    YoYoReactor::synchronize()
    {

      bool nearXY = false, nearZ = false;
      Variable v;
      int secs_at_surface = 60;

      if (m_lastRefState.hasAttribute("near_z"))
      {

        v = m_lastRefState.getAttribute("near_z");
        BooleanDomain const &nearz = dynamic_cast<BooleanDomain const &>(v.domain());

        if (nearz.isSingleton())
        {
          nearZ = nearz.getStringSingleton() == "true" || nearz.getStringSingleton() == "1";
        }

      }

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
        m_time_underwater ++;
        m_time_at_surface  = 0;
        std::cerr << "[YOYO] ASCEND" << std::endl;

        if (nearXY)
        {
          requestReference(m_lat, m_lon, m_speed, 0);
          state = SURFACE;
        }
        else if (nearZ)
        {
          if (m_time_underwater >= m_secs_underwater)
          {
            requestReference(m_lat, m_lon, m_speed, 0);
            state = SURFACE;
          }
          else
          {
            requestReference(m_lat, m_lon, m_speed, m_maxz);
            state = DESCEND;
          }
        }
        break;

        case (DESCEND):
                    m_time_underwater ++;
        m_time_at_surface  = 0;
        std::cerr << "[YOYO] DESCEND" << std::endl;
        if (nearXY)
        {
          requestReference(m_lat, m_lon, m_speed, 0);
          state = SURFACE;
        }
        else if (nearZ)
        {
          requestReference(m_lat, m_lon, m_speed, m_minz);
          state = ASCEND;
        }
        break;

        case (SURFACE):
        m_time_underwater = 0;
        std::cerr << "[YOYO] SURFACE" << std::endl;
        if (nearXY && nearZ)
        {
          Observation obs = Observation(s_yoyo_tl, "Done");
          obs.restrictAttribute("latitude", FloatDomain(m_lat, m_lat));
          obs.restrictAttribute("longitude", FloatDomain(m_lon, m_lon));
          obs.restrictAttribute("speed", FloatDomain(m_speed, m_speed));
          obs.restrictAttribute("max_z", FloatDomain(m_maxz, m_maxz));
          obs.restrictAttribute("min_z", FloatDomain(m_minz, m_minz));
          postUniqueObservation(obs);
          state = IDLE;
        }
        else if (nearZ)
        {
          m_time_at_surface ++;
          if (m_time_at_surface >= secs_at_surface)
          {
            requestReference(m_lat, m_lon, m_speed, m_maxz);
            state = DESCEND;
          }
        }
        break;
        default:
          std::cerr << "[YOYO] IDLE" << std::endl;
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

      std::cerr << "[YOYO] Request(" << lat << ", " << lon << ", " << speed << ", " << z << std::endl;

      postGoal(g);
    }

    void
    YoYoReactor::handleRequest(TREX::transaction::goal_id const &goal)
    {
      std::cerr << "[YOYO] handleRequest(" << *(goal.get()) << ")" << std::endl;

      if ( s_trex_pred != m_lastControl.predicate() )
      {
        std::cerr << "[YOYO] won't handle this request because TREX is not controlling the vehicle!" << std::endl;
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

        v = g->getAttribute("secs_underwater");
        if (v.domain().isSingleton())
          m_secs_underwater = v.domain().getTypedSingleton<int, true>();

        state = DESCEND;
        requestReference(m_lat, m_lon, m_speed, m_maxz);
        postUniqueObservation(*g);

      }
      else
      {
        std::cerr << "[YOYO] request not valid" << std::endl;
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
      std::cerr << "[YOYO] notify(" << obs << ")" << std::endl;

      // std::string timeline = obs.object().str();
      //std::string predicate = obs.predicate().str();
    
      if (s_reference_tl == obs.object())
        m_lastReference = obs;
      else if (s_refstate_tl == obs.object())
        m_lastRefState = obs;
      else if (s_control_tl == obs.object())
        m_lastControl = obs;
    }

    YoYoReactor::~YoYoReactor()
    {
      // TODO Auto-generated destructor stub
    }
  }
}
