#include <trex/utils/Plugin.hh>
#include <trex/utils/LogManager.hh>

namespace {
  
  
  TREX::utils::SingletonUse<TREX::utils::LogManager> s_log;
  
}

namespace TREX {
  
  void initPlugin() {
    ::s_log->syslog("plugin.mabri.europa")<<"Europa with MBARI extensions loaded."
					  <<std::endl;
  }

}
