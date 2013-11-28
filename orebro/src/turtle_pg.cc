#include <trex/utils/LogManager.hh>
#include <trex/utils/Plugin.hh>

using namespace TREX::utils;

namespace {
  // A log entry for displaying global messages to TREX.log
  SingletonUse<LogManager> s_log;
}

namespace TREX {

  // The function TREX call when loading a plugin
  void initPlugin() {
    ::s_log->syslog("plugin.TurtleSimPub")<<"TurtleSimPub loaded."<<std::endl;
  }

}
