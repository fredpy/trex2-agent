#include "DrifterTracker.hh"

using namespace mbari;

/*
 * class mbari::MessageHandler
 */
MessageHandler::MessageHandler(MessageHandler::xml_arg const &arg) 
  :m_exchange(TREX::utils::parse_attr<std::string>(factory::node(arg), 
						   "exchange")),
   m_route(TREX::utils::parse_attr<std::string>("", factory::node(arg), "route")),
   m_tracker(*(arg.second)) {}

bool MessageHandler::provide(std::string const &timeline, bool control) {
  m_tracker.provide(timeline, control);
  if( m_tracker.isInternal(timeline) ) {
    if( control )
      m_tracker.goalHandler(timeline, this);
    return true;
  }
  return false;
}

void MessageHandler::notify(TREX::transaction::Observation const &obs) {
  m_tracker.postObservation(obs);
}

TREX::transaction::TICK MessageHandler::now() const {
  return m_tracker.getCurrentTick();
}

double MessageHandler::tickToTime(TREX::transaction::TICK date) const {
  return m_tracker.tickToTime(date);
}

double MessageHandler::tickDuration() const {
  return m_tracker.tickDuration();
}

