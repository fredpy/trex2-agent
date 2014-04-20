#include <trex/utils/Plugin.hh>
#include "DoradoReactor.hh"

using namespace TREX::mbari;

namespace {

  /** @brief TREX log entry point */
  TREX::utils::SingletonUse<TREX::utils::LogManager> s_log;

  // declaration to xml factory
  
  TREX::transaction::TeleoReactor::xml_factory::declare<DoradoReactor> decl("DoradoReactor");
}

namespace TREX {
  
  void initPlugin() {
    ::s_log->syslog("plugin.dorado", transaction::info)<<"MBARI dorado plugin loaded.";
  }

} // TREX
