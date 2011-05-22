#include "EuropaReactor.hh"

#include "Plugin.hh"
#include "LogManager.hh"

using namespace TREX::europa;
using namespace TREX::transaction;
using namespace TREX::utils;

namespace {

  SingletonUse<LogManager> s_log;

  TeleoReactor::xml_factory::declare<EuropaReactor> decl("EuropaReactor");

}

namespace TREX {

  void initPlugin() {
    ::s_log->syslog("plugin.europa")<<"Europa loaded."<<std::endl;
    // ::decl;
  }  

}
