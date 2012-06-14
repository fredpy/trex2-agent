#ifndef H_DuneClock
# define H_DuneClock

# include <trex/agent/Clock.hh>

# include <boost/thread/recursive_mutex.hpp>


namespace TREX {
  namespace LSTS {
  
    class DuneClock :public TREX::agent::Clock {
    public:
      explicit DuneClock(boost::property_tree::ptree::value_type &node);
      ~DuneClock();
      
      TREX::transaction::TICK getNextTick();
      bool free() const;
      double tickDuration() const {
        return m_floatTick;
      }
      
      TREX::transaction::TICK timeToTick(time_t secs, suseconds_t usec=0) const;
      double  tickToTime(TREX::transaction::TICK cur) const;
      std::string date_str(TREX::transaction::TICK &tick) const;
        
    private:
      void start();
      double getSleepDelay() const;

      double timeLeft() const;
      
      void setNextTickDate(unsigned factor);
      
      double get() const;
      
      double m_floatTick;
      
      typedef boost::recursive_mutex mutex_type;
      mutable mutex_type m_lock;
      
      bool m_started;
      TREX::transaction::TICK m_tick;
      double m_nextTickDate;

    }; // TREX::LSTS::DuneClock
  
    
  } // TREX::LSTS
} // TREX

#endif // H_DuneClock
