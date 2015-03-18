#include <trex/utils/LogManager.hh>
#include <trex/utils/Plugin.hh>

using namespace TREX::utils;

namespace {
  // A log entry for displaying global messages to TREX.log
  SingletonUse<LogManager> s_log;
}

namespace TREX {

  // The function TREX call when loading a plugin
  // Any T-REX plugin must to implement it
  // As a bonus you can use it to do some global intialization if needed
  void initPlugin() {

    // write this message in $TREX_LOG_DIR/latest/TREX.log
    ::s_log->syslog("plugin.TurtleSimPub")<<"TurtleSimPub loaded."<<std::endl;

    
  }

}
