#include "DuneClock.hh"

#include <Dune/Time/Clock.hpp>

using namespace TREX::LSTS;
using TREX::agent::Clock;

namespace {
  Clock::xml_factory::declare<steady_clock> default_decl("DuneClock");
  Clock::xml_factory::declare<steady_clock> steady_decl("DuneSteady");
  Clock::xml_factory::declare<posix_clock> posix_decl("DunePosix"); 
}

/*
 * class TREX::LSTS::dune_posix_clock
 */
dune_posix_clock::time_point dune_posix_clock::now() {
  return time_point(duration(Dune::Time::Clock::getSinceEpochNsec()));
}

/*
 * class TREX::LSTS::dune_steady_clock
 */
dune_steady_clock::time_point dune_steady_clock::now() {
  return time_point(duration(Dune::Time::Clock::getNsec()));
}
