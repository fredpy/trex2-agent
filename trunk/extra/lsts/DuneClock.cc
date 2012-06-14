#include "DuneClock.hh"

#include <cmath>
#include <Dune/Time/Clock.hpp>

#include <trex/utils/TimeUtils.hh>
#include <trex/utils/LogManager.hh>

#include <boost/date_time/posix_time/posix_time.hpp>


using namespace TREX::LSTS;
using namespace TREX::utils;
using TREX::transaction::TICK;

namespace bt = boost::posix_time;
namespace bd = boost::date_time;

namespace {
  SingletonUse<LogManager> s_log;
  
  TREX::agent::Clock::xml_factory::declare<DuneClock> decl("DuneClock");
}

/*
 * class TREX::LSTS::DuneClock
 */ 

// structors 

DuneClock::DuneClock(boost::property_tree::ptree::value_type &node)
  :TREX::agent::Clock(0.0001),
   m_floatTick(parse_attr<double>(node, "tick")),
   m_started(false), m_tick(0) {
  if( m_floatTick<=0.0 )
    throw XmlError(node, "Negative duration in tick attribute.");
}

DuneClock::~DuneClock() {}

// observers

bool DuneClock::free() const {
  return true;
}
      
double DuneClock::getSleepDelay() const {
  if( m_started ) {
    double delay;
    TICK tick;
    {
      mutex_type::scoped_lock guard(m_lock);
      delay = timeLeft();
      tick = m_tick;
    }
    if( delay<0.0 ) {
      delay = -delay;
      if( delay>0.05*tickDuration() ) {
        std::ostringstream oss;
        oss<<"Clock]["<<m_tick<<"][WARNING";
        s_log->syslog(oss.str())<<delay<<" secs late before sleep";
      }
      delay = 0.0;
    }
    return delay;
  } else 
    return TREX::agent::Clock::getSleepDelay();
}

TICK DuneClock::timeToTick(time_t secs, suseconds_t usec) const {
  double val, delta;
  TICK current;
  val = usec;
  val *= 1e-9;
  val += secs;
  {
    mutex_type::scoped_lock guard(m_lock);
    current = m_tick+1;
    delta = val - m_nextTickDate;
  }
  delta /= tickDuration();
  current += std::floor(delta+0.5);
  return current; 
}

double  DuneClock::tickToTime(TICK cur) const {
  TICK current;
  double next;
  {
    mutex_type::scoped_lock guard(m_lock);
    next = m_nextTickDate;
    current = m_tick+1;
  }
  double d_tick = cur-current;
  d_tick *= tickDuration();
  return next+d_tick;
}

std::string DuneClock::date_str(TICK &tick) const {
  double date = tickToTime(tick);
  time_t secs = floor(date);
  date -= secs;  
  unsigned short msecs = floor(1000.*date);
        
  bt::ptime udate = bt::from_time_t(secs) + bt::millisec(msecs);
  
  std::ostringstream oss;
  oss<<udate;
  return oss.str();
}

double DuneClock::timeLeft() const {
  return m_nextTickDate - Dune::Time::Clock::getSinceEpoch();
}

// manipulators

double DuneClock::get() const {
  return Dune::Time::Clock::getSinceEpoch();
}


void DuneClock::setNextTickDate(unsigned factor) {
  m_nextTickDate += m_floatTick*factor;
}

void DuneClock::start() {
  m_nextTickDate = get();
  setNextTickDate(1);
  m_started = true;
}

TICK DuneClock::getNextTick() {
  mutex_type::scoped_lock guard(m_lock);
  if( m_started ) {
    double howLate = -timeLeft();
    if( howLate>=0.0 ) {
      double ratio = howLate/m_floatTick;
      int incr = 1+static_cast<int>(std::floor(ratio));
      
      m_tick += incr;
      setNextTickDate(incr);
      if( ratio>=0.1 ) {
        std::ostringstream oss;
        oss<<"Clock]["<<m_tick<<"][WARNING";
        s_log->syslog(oss.str())<<howLate<<" secs late.";
      }
    }
  }
  return m_tick;
}
