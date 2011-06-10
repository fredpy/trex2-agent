# include "TREXversion.hh"

#ifndef TREX_MAJOR // needed as jam does not generate version.hh
# include <trex/version.hh>
#endif 

# include <sstream>


unsigned short TREX::version::major() {
  return TREX_MAJOR;
}

unsigned short TREX::version::minor() {
  return TREX_MINOR;
}

unsigned short TREX::version::release() {
  return TREX_PATCH;
}

unsigned long TREX::version::number() {
  unsigned long version = major();
  version = 100*version + minor();
  version = 100*version + release();
  return version;
}

std::string TREX::version::str() {
  std::ostringstream oss;
  oss<<major()<<'.'<<minor()<<'.'<<release();
  return oss.str();
}
