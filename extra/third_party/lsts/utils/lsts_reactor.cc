#include "lsts_reactor.hh"

using namespace trex_lsts;

using TREX::transaction::Observation;
using TREX::transaction::TeleoReactor;

/*
 * class trex_lsts::lsts_reactor
 */

// structors

lsts_reactor::lsts_reactor(xml_arg_type arg):TeleoReactor(arg, false) {}

lsts_reactor::~lsts_reactor() {}

// observers

bool lsts_reactor::is_fresh(Observation const &obs) const {
  obs_cache::const_iterator pos = m_posted_obs.find(obs.object());
  return m_posted_obs.end()==pos || !pos->second->consistentWith(obs);
}

// manipulators

bool lsts_reactor::post_unique(Observation const &obs, bool verbose) {
  obs_cache::const_iterator pos = m_posted_obs.lower_bound(obs.object());
  if( m_posted_obs.end()==pos || !pos->second->consistentWith(obs) ) {
    // If timeline was not previously owned try to provide it
    if( !( isInternal(obs.object()) || isExternal(obs.object()) ) )
      provide(obs.object(), false);
    m_posted_obs[obs.object()] = MAKE_SHARED<Observation>(obs);
    postObservation(obs, verbose);
    return true;
  }
  return false;
}


