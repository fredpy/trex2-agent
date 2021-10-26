#ifndef H_FastClock
# define H_FastClock

# include "Agent.hh"

namespace TREX {
  namespace agent {
    
    
    class FastClock: public Clock {
    public:
      using typename Clock::duration_type;
      using typename Clock::date_type;
      
      FastClock(clock_ref baseline,
                date_type const &epoch,
                duration_type const &duration,
		bool no_skip = true);
      explicit FastClock(boost::property_tree::ptree::value_type &node);
      virtual ~FastClock();
      
      TREX::transaction::TICK max_tick() const;
      TREX::transaction::TICK initialTick() const {
        return m_clock->initialTick();
      }
      date_type epoch() const {
        return m_epoch;
      }
      duration_type tickDuration() const {
        return m_freq;
      }
      
      std::string date_str(TREX::transaction::TICK const &tick) const;
      std::string duration_str(TREX::transaction::TICK dur) const;
      
      
      std::string info() const;
      
    private:
      clock_ref     m_clock;
      date_type     m_epoch;
      duration_type m_freq;
      bool const    m_no_skip;
      std::optional<transaction::TICK> m_real_prev;
      transaction::TICK m_prev;
      
      duration_type getSleepDelay() const {
        return m_clock->getSleepDelay();
      }
      
      TREX::transaction::TICK getNextTick();
      
      void start() {
        m_clock->start();
      }
      
      bool free() const {
        return m_clock->free();
      }
      
      size_t count() const {
        return m_clock->count();
      }
      
      duration_type doSleep() {
        return m_clock->doSleep(); // not sure if I need to convert to warped time
      }
    };
    
  }
}

#endif
