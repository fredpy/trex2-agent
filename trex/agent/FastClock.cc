#include "FastClock.hh"

using namespace TREX::agent;
using TREX::transaction::TICK;
namespace utils=TREX::utils;
namespace bpt=boost::property_tree;

namespace {
  Clock::xml_factory::declare<FastClock> fast_decl("FastClock");
}

FastClock::FastClock(clock_ref baseline,
                     date_type const &epoch,
                     duration_type const &duration)
:Clock(duration_type::zero()), m_clock(baseline), m_epoch(epoch), m_freq(duration) {
}

FastClock::FastClock(bpt::ptree::value_type &node)
:Clock(duration_type::zero()) {
  // TODO: implement the parsing
  throw Clock::Error("xml parsing for FastClock not implemented");
}

FastClock::~FastClock() {}


TICK FastClock::max_tick() const {
  TICK base = m_clock->max_tick(),
    mine = timeToTick(date_type(boost::posix_time::max_date_time));
  
  // Take the smallest value between the maximum tick of the base clock and
  // the maximum tick that can be represented by our simulated time warp
  return std::min(base, mine);
}

std::string FastClock::date_str(TICK const &tick) const {
  return  boost::posix_time::to_iso_extended_string(tickToTime(tick));
}

std::string FastClock::duration_str(TICK dur) const {
  duration_type dt = tickDuration()*dur;
  typedef TREX::utils::chrono_posix_convert<duration_type> cvt;

  typename cvt::posix_duration p_dur = cvt::to_posix(dt);
  
  std::ostringstream oss;
  oss<<p_dur;
  return oss.str();
}

std::string FastClock::info() const {
  std::ostringstream oss;
  
  oss<<"Warped clock:\n"
    << " - sim epoch: "<<epoch()
    << "\n - sim freq: "<<duration_str(1)
    << "\n - base: "<<m_clock->info();
  return oss.str();
}
