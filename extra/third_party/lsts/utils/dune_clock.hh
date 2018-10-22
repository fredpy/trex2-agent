#ifndef H_TREX_dune_dune_clock
# define H_TREX_dune_dune_clock

# include <trex/agent/RealTimeClock.hh>

namespace trex_lsts {
  
  struct dune_posix_clock {
    typedef CHRONO::nanoseconds duration;
    typedef duration::rep       rep;
    typedef duration::period    period;
    typedef CHRONO::time_point<dune_posix_clock> time_point;
    
    static const bool is_steady = false;
    
    static time_point now();
    
  }; // trex_lsts::dune_posix_clock
  
  
  struct dune_steady_clock {
    typedef CHRONO::nanoseconds duration;
    typedef duration::rep       rep;
    typedef duration::period    period;
    typedef CHRONO::time_point<dune_steady_clock> time_point;
    
    static const bool is_steady = true;
    
    static time_point now();
  }; // trex_lsts::dune_steady_clock
  
  
  typedef TREX::agent::rt_clock<CHRONO_NS::milli, dune_posix_clock> posix_clock;
  typedef TREX::agent::rt_clock<CHRONO_NS::milli, dune_steady_clock> steady_clock;
  
} // trex_lsts

#ifndef CPP11_HAS_CHRONO

namespace boost {
  namespace chrono {
    
    /** @brief Information about TREX::LSTS::dune_posix
     *
     * Textual information for the dune posix clock
     *
     * @relates TREX::LSTS::dune_posix
     *
     * @author Frederic Py
     */
    template<class CharT>
    struct clock_string<trex_lsts::dune_posix_clock, CharT> {
      /** @brief Clock name */
      static std::basic_string<CharT> name() {
        static const CharT u[] =
        { 'D', 'u', 'n', 'e', '_', 'p', 'o', 's', 'i', 'x', '_', 'c', 'l', 'o', 'c', 'k' };
        static const std::basic_string<CharT> str(u, u + sizeof(u) / sizeof(u[0]));
        return str;
      }
      /** @brief Clock epoch */
      static std::basic_string<CharT> since() {
        static const CharT u[] =
        { ' ', 's', 'i', 'n', 'c', 'e', ' ', 'J', 'a', 'n', ' ', '1', ',', ' ', '1', '9', '7', '0' };
        static const std::basic_string<CharT> str(u, u + sizeof(u) / sizeof(u[0]));
        return str;
      }
    }; // boost::chrono::clock_string<TREX::LSTS::dune_posix_clock, >
    
    /** @brief Information about TREX::LSTS::dune_steady
     *
     * Textual information for the dune steady clock
     *
     * @relates TREX::LSTS::dune_steady
     *
     * @author Frederic Py
     */
    template<class CharT>
    struct clock_string<trex_lsts::dune_steady_clock, CharT> {
      /** @brief Clock name */
      static std::basic_string<CharT> name() {
        static const CharT u[] =
        { 'D', 'u', 'n', 'e', '_', 's', 't', 'e', 'a', 'd', 'y', '_', 'c', 'l', 'o', 'c', 'k' };
        static const std::basic_string<CharT> str(u, u + sizeof(u) / sizeof(u[0]));
        return str;
      }
      /** @brief Clock epoch */
      static std::basic_string<CharT> since() {
        static const CharT u[] =
        { ' ', 's', 'i', 'n', 'c', 'e', ' ', 'b', 'o', 'o', 't' };
        static const std::basic_string<CharT> str(u, u + sizeof(u) / sizeof(u[0]));
        return str;
      }
    }; //boost::chrono::clock_string<TREX::LSTS::dune_posix_clock, >
  } //boost::chrono
} // boost
#endif // !CPP11_HAS_CHRONO


#endif // H_TREX_dune_dune_clock
