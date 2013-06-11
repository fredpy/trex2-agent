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

    YoYoReactor::YoYoReactor(TeleoReactor::xml_arg_type arg) :
    LstsReactor(arg),
      m_lastRefState("refstate", "Failed"),
      m_lastReference("reference", "Failed"),
      m_lastControl("control", "Failed")
    {
      m_lat = m_lon = m_speed = m_minz = m_maxz = -1;
      m_secs_underwater = 0;
      state = IDLE;
      use("reference", true);
      use("refstate", false);
      use("control");
      provide("yoyo");
    }

    void
    YoYoReactor::handleInit()
    {
      Observation yoyo("yoyo", "Idle");
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
      switch(state)
      {
        case (ASCEND):
          postUniqueObservation(Observation("yoyo", "Exec"));
          break;
        case (DESCEND):
          postUniqueObservation(Observation("yoyo", "Exec"));
          break;
        case (SURFACE):
          postUniqueObservation(Observation("yoyo", "Exec"));
          break;
        default:
        case (IDLE):
          postUniqueObservation(Observation("yoyo", "Idle"));
          break;
      }
       std::cerr << "[YOYO] synchronize" << std::endl;
    }

//    void
//    YoYoReactor::requestReference(Observation & ref)
//    {
//
//    }

    void
    YoYoReactor::handleRequest(TREX::transaction::goal_id const &g)
    {
      std::cerr << "[YOYO] handleRequest(" << *(g.get()) << ")" << std::endl;
    }

    void
    YoYoReactor::handleRecall(TREX::transaction::goal_id const &g)
    {
      std::cerr << "[YOYO] handleRecall(" << *(g.get()) << ")" << std::endl;
    }

    void
    YoYoReactor::notify(TREX::transaction::Observation const &obs)
    {
      std::cerr << "[YOYO] notify(" << obs << ")" << std::endl;

      std::string timeline = obs.object().str();
      std::string predicate = obs.predicate().str();

      if (timeline == "reference")
        m_lastReference = obs;
      else if (timeline == "refstate")
        m_lastRefState = obs;
      else if (timeline == "control")
        m_lastControl = obs;
    }

    YoYoReactor::~YoYoReactor()
    {
      // TODO Auto-generated destructor stub
    }
  }
}
