#include "DrifterTracker.hh"

namespace {

  /** @brief TREX log entry point */
  TREX::utils::SingletonUse<TREX::utils::LogManager> s_log;

}

namespace TREX {
  
  void initPlugin() {
    ::s_log->syslog("plugin.tracker")<<"MBARI tracker loaded."<<std::endl;
    // ::decl;
  }

} // TREX
