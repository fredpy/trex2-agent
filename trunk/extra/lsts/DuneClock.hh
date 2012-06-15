#ifndef H_DuneClock
# define H_DuneClock

# include <trex/agent/RealTimeClock.hh>

namespace TREX {
  namespace LSTS {
  
    class dune_posix_clock {
    public:
      typedef boost::chrono::nanoseconds duration;
      typedef duration::rep              rep;
      typedef duration::period           period;
      typedef boost::chrono::time_point<dune_posix_clock> time_point;

      static const bool is_steady = false;
      
      static time_point now();
            
    }; // TREX::LSTS::dune_posix_clock
    
    typedef agent::rt_clock<boost::nano, dune_posix_clock> posix_clock;

    class dune_steady_clock {
    public:
      typedef boost::chrono::nanoseconds                   duration;
      typedef duration::rep                                rep;
      typedef duration::period                             period;
      typedef boost::chrono::time_point<dune_steady_clock> time_point;

      static const bool is_steady = true;
      
      static time_point now();
      
    }; // TREX::LSTS::dune_steady_clock  

    typedef agent::rt_clock<boost::nano, dune_steady_clock> steady_clock;

  } // TREX::LSTS
} // TREX

#endif // H_DuneClock
