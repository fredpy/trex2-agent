#include "Constraints.hh"

#include <extra/europa/Assembly.hh>
#include <trex/utils/Plugin.hh>
#include <trex/utils/LogManager.hh>

namespace {
  
  class MBARIPlugin :public TREX::europa::SchemaPlugin {
  public:
    void registerComponents(TREX::europa::Assembly const &assembly) {
      TREX_REGISTER_CONSTRAINT(assembly,mbari::europa::GeoUTMConstraint, 
			       geo_to_utm, trex);
    }
  };

  MBARIPlugin europa_extensions;
  
  TREX::utils::SingletonUse<TREX::utils::LogManager> s_log;
  
}

namespace TREX {
  
  void initPlugin() {
    ::s_log->syslog("plugin.mabri.europa")<<"Europa with MBARI extensiosn loaded."
					  <<std::endl;
  }

}
