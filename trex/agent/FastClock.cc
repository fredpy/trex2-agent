#include "FastClock.hh"

using namespace TREX::agent;
using TREX::transaction::TICK;
namespace utils=TREX::utils;
namespace tlog=utils::log;
namespace bpt=boost::property_tree;

namespace {
  Clock::xml_factory::declare<FastClock> fast_decl("FastClock");
}

FastClock::FastClock(clock_ref baseline,
                     date_type const &epoch,
                     duration_type const &duration,
		     bool no_skip)
:Clock(duration_type::zero()), m_clock(baseline), m_epoch(epoch),
 m_freq(duration), m_no_skip(no_skip) {
}

FastClock::FastClock(bpt::ptree::value_type &node)
  :Clock(duration_type::zero()),
   m_epoch(utils::parse_attr<date_type>(node, "epoch")),
   m_no_skip(!utils::parse_attr<bool>(false, node, "skip")) {
  utils::SingletonUse<Clock::xml_factory> clk_f;
  
  std::chrono::nanoseconds ns_tick = std::chrono::nanoseconds::zero();
  typedef std::optional< typename std::chrono::nanoseconds::rep >
    value_type;
  value_type value;
  bool has_attr = false;
  
  // Get nanoseconds
  value = utils::parse_attr<value_type>(node, "nanos");
  if( value ) {
    has_attr = true;
    ns_tick += std::chrono::nanoseconds(*value);
  }
  
  // Get microseconds
  value = utils::parse_attr<value_type>(node, "micros");
  if( value ) {
    has_attr = true;
    ns_tick += std::chrono::microseconds(*value);
  }
  
  // Get milliseconds
  value = utils::parse_attr<value_type>(node, "millis");
  if( value ) {
    has_attr = true;
    ns_tick += std::chrono::milliseconds(*value);
  }
  
  // Get seconds
  value = utils::parse_attr<value_type>(node, "seconds");
  if( value ) {
    has_attr = true;
    ns_tick += std::chrono::seconds(*value);
  }
  
  // Get minutes
  value = utils::parse_attr<value_type>(node, "minutes");
  if( value ) {
    has_attr = true;
    ns_tick += std::chrono::minutes(*value);
  }
  
  // Get hours
  value = utils::parse_attr<value_type>(node, "hours");
  if( value ) {
    has_attr = true;
    ns_tick += std::chrono::hours(*value);
  }

  if( !has_attr ) {
    std::ostringstream oss;
    oss<<"No tick duration attribute found. Specify one or more of the following:\n"
       <<" - hours,minutes,seconds,millis,nanos";
    throw utils::XmlError(node, oss.str());
  }
  
  m_freq = std::chrono::duration_cast<duration_type>(ns_tick);

  // Extract the clock
  boost::property_tree::ptree::iterator i = node.second.begin();
  if( !clk_f->iter_produce(i, node.second.end(), m_clock) )
    throw utils::XmlError(node, "Missing clock definition for clock usedd by FastClock");

}

FastClock::~FastClock() {}


TICK FastClock::max_tick() const {
  TICK base = m_clock->max_tick(), mine;
  typedef typename std::chrono::duration< double, std::ratio<1> > tick_rate;
  typedef utils::chrono_posix_convert<tick_rate> convert;

  double seconds = convert::to_chrono(date_type(boost::posix_time::max_date_time)-epoch()).count();

  seconds /= std::chrono::duration_cast<tick_rate>(m_freq).count(); 
  mine = std::floor(seconds);
  
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

TICK FastClock::getNextTick() {
  TICK cur = m_clock->getNextTick();

  if( m_no_skip ) {
    if( m_real_prev ) {
      
      if( (*m_real_prev)<cur ) {
	TICK delta = cur - (*m_real_prev); 
	m_prev += 1;
	if( delta>1 ) {
	  syslog(tlog::info)<<"Hiding that the real clock skipped "
			    <<(delta-1)<<" tick(s)\n\t"
			    <<"- real_tick = "<<cur
			    <<"\n\t- sim tick = "<<m_prev;
	}
      }
    } else
      m_prev = cur;
    m_real_prev = cur;
    return m_prev;
  }
  return cur;
}


std::string FastClock::info() const {
  std::ostringstream oss;
  
  oss<<"Warped clock:\n"
    << " - sim epoch: "<<epoch()
    << "\n - sim freq: "<<duration_str(1)
    << "\n - hide_skips: "<<m_no_skip
    << "\n - base: "<<m_clock->info();
  return oss.str();
}
