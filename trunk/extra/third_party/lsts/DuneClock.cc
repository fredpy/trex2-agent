#include "DuneClock.hh"

#include <Dune/Time/Clock.hpp>

using namespace TREX::LSTS;
using TREX::agent::Clock;

namespace {
  /** @brief Dune steady clock definition
   *
   * Definition of the steady dune clock for TREX
   * The XML definition in a mission file would be as follow:
   * @code 
   *  <DuneClock seconds="1" />
   * @endcode
   * Or for a 2Hz clock and using the alternate tag
   * @code
   *  <DuneSteady millis="500" /> 
   * @endcode 
   * @relates TREX::LSTS::steady_clock
   * @{
   */
  Clock::xml_factory::declare<steady_clock> default_decl("DuneClock");
  Clock::xml_factory::declare<steady_clock> steady_decl("DuneSteady");
  /** @} */
  
  /** @brief Dune posix clock definition
   *
   * Definition of the posix dune clock for TREX
   * The XML definition in a mission file would be as follow:
   * @code 
   *  <DunePosix seconds="1" />
   * @endcode
   * @relates TREX::LSTS::steady_clock
   */
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
