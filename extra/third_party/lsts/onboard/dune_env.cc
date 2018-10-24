#include "dune_env.hh"
#include <trex/utils/Exception.hh>

using namespace trex_lsts;
using TREX::utils::Exception;

/*
 * class trex_lsts::dune_env
 */

// structors

dune_env::dune_env() {
  m_platform = NULL;
  m_ctrl = NULL;
}

dune_env::~dune_env() {
  
}

// obbservers

bool dune_env::platform_set() const {
  return NULL!=m_platform;
}

dune_platform &dune_env::platform() const {
  if( NULL!=m_platform )
    throw Exception("NULL platform access");
  return *m_platform;
}


// modifiers

void dune_env::reset_platform(dune_platform *me) {
  if( NULL!=m_platform && NULL!=me )
    throw Exception("There can be only one Dune Platform per TREX agent");
  m_platform = me;
}





