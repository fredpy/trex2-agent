#include "DTAReactor.hh"
#include <trex/utils/Plugin.hh>

using namespace TREX::transaction;
using namespace mbari::iridium;

namespace {

  /** @brief TREX log entry point */
  TREX::utils::singleton_use<TREX::utils::LogManager> s_log;
  TeleoReactor::xml_factory::declare<DTAReactor> decl("DTAiridium");  

}

namespace TREX {
  
  void initPlugin() {
    ::s_log->syslog("plugin.iridium", info)<<"MBARI iridium mailer loaded.";
  }

} // TREX
