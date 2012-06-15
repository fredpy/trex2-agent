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
    
    typedef agent::rt_clock<boost::milli, dune_posix_clock> posix_clock;

    class dune_steady_clock {
    public:
      typedef boost::chrono::nanoseconds                   duration;
      typedef duration::rep                                rep;
      typedef duration::period                             period;
      typedef boost::chrono::time_point<dune_steady_clock> time_point;

      static const bool is_steady = true;
      
      static time_point now();
      
    }; // TREX::LSTS::dune_steady_clock  

    typedef agent::rt_clock<boost::milli, dune_steady_clock> steady_clock;

  } // TREX::LSTS
} // TREX

namespace boost {
  namespace chrono {
  
    template<class CharT>
    struct clock_string<TREX::LSTS::dune_posix_clock, CharT> {
      static std::basic_string<CharT> name() {
        static const CharT u[] =  
          { 'D', 'u', 'n', 'e', '_', 'p', 'o', 's', 'i', 'x', '_', 'c', 'l', 'o', 'c', 'k' };
        static const std::basic_string<CharT> str(u, u + sizeof(u) / sizeof(u[0]));
        return str;
      } 
      static std::basic_string<CharT> since() {
        static const CharT u[] =
          { ' ', 's', 'i', 'n', 'c', 'e', ' ', 'J', 'a', 'n', ' ', '1', ',', ' ', '1', '9', '7', '0' };
        static const std::basic_string<CharT> str(u, u + sizeof(u) / sizeof(u[0]));
        return str;
      }
    }; // boost::chrono::clock_string<TREX::LSTS::dune_posix_clock, >
    
    template<class CharT>
    struct clock_string<TREX::LSTS::dune_steady_clock, CharT> {
      static std::basic_string<CharT> name() {
        static const CharT u[] =  
          { 'D', 'u', 'n', 'e', '_', 's', 't', 'e', 'a', 'd', 'y', '_', 'c', 'l', 'o', 'c', 'k' };
        static const std::basic_string<CharT> str(u, u + sizeof(u) / sizeof(u[0]));
        return str;
      } 
      static std::basic_string<CharT> since() {
        static const CharT u[] =
          { ' ', 's', 'i', 'n', 'c', 'e', ' ', 'b', 'o', 'o', 't' };
        static const std::basic_string<CharT> str(u, u + sizeof(u) / sizeof(u[0]));
        return str;
      }
    }; // boost::chrono::clock_string<TREX::LSTS::dune_posix_clock, >
  } // boost::chrono
} // boost

#endif // H_DuneClock
