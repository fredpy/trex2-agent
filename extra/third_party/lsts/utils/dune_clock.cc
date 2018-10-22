#include "dune_clock.hh"

#include <DUNE/Time/Clock.hpp>

using namespace trex_lsts;
using TREX::agent::Clock;

namespace {
  typedef steady_clock default_clock;
  
  Clock::xml_factory::declare<posix_clock> posix_decl("DunePosix");
  Clock::xml_factory::declare<steady_clock> steady_decl("DuneSteady");

  Clock::xml_factory::declare<default_clock> default_decl("DuneClock");
}

/*
 * class trex_lsts::dune_posix_clock
 */

// statics

dune_posix_clock::time_point dune_posix_clock::now() {
  return time_point(duration(DUNE::Time::Clock::getSinceEpochNsec()));
}

/*
 * class trex_lsts::dune_steady_clock
 */

// statics

dune_steady_clock::time_point dune_steady_clock::now() {
  return time_point(duration(DUNE::Time::Clock::getNsec()));
}
