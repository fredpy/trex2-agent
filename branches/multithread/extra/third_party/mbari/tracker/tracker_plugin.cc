#include "DrifterTracker.hh"

#include <trex/utils/Plugin.hh>


namespace {

  /** @brief TREX log entry point */
  TREX::utils::singleton_use<TREX::utils::LogManager> s_log;

}

namespace TREX {
  
  void initPlugin() {
    ::s_log->syslog("plugin.dta", transaction::info)<<"MBARI assets tracker loaded.";
  }

} // TREX
