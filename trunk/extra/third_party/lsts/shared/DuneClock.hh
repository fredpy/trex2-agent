#ifndef H_DuneClock
# define H_DuneClock

# include <trex/agent/RealTimeClock.hh>

namespace TREX {
  namespace LSTS {
  
    /** @brief dune posix clock
     *
     * The interface to dune implementation of the posix clock. 
     * This interface respect the boost::chrono definition 
     * allowing user to define their own rt_clock for TREX
     *
     * @sa TREX::agent::rt_clock
     * @sa dune_steady_clock
     *
     * @author Frederic Py
     */
    class dune_posix_clock {
    public:
      /** @brief duration type
       * 
       * The type used to for duration by this clock
       */
      typedef CHRONO::nanoseconds duration;
      /** @brief duration storage type
       * 
       * The type used to count ticks by this clock
       */
      typedef duration::rep              rep;
      /** @brief clock period
       * 
       * the period of this clock. 
       */
      typedef duration::period           period;
      /** @brief Clock time point
       *
       * The type to represent a time point (or date) for this
       * clock.
       */
      typedef CHRONO::time_point<dune_posix_clock> time_point;

      /** @brief Steadyness flag
       *
       * The flag that indicate if this clock is steady or not. 
       * A non steady clock do not guarantee that all the 
       * timepoints will be ordered (for example when the clock 
       * time is reset to an anterior date).  
       */
      static const bool is_steady = false;
      
      /** @brief Current date
       *
       * @return the current time point
       */
      static time_point now();
            
    }; // TREX::LSTS::dune_posix_clock
    
    /** @brief TREX dune posix clock
     *
     * The default TREX implemenation for the dune posix clock. 
     * The period of this clock is set to 1000Hz to allow some 
     * flexibility on the duration of a tick while not using to 
     * much precision (There's no need to run TREX at a tick rate
     * higher than 1000Hz  
     *
     * @relates dune_posix_clock
     * @sa steady_clock
     */
    typedef agent::rt_clock<CHRONO_NS::milli, dune_posix_clock> posix_clock;

    /** @brief dune steady clock
     *
     * The interface to dune implementation of the steady clock. 
     * This interface respect the boost::chrono definition allowing 
     * user to define their own rt_clock for TREX
     *
     * @sa TREX::agent::rt_clock
     * @sa dune_posix_clock
     *
     * @author Frederic Py
     */
    class dune_steady_clock {
    public:
      /** @brief duration type
       * 
       * The type used to for duration by this clock
       */
      typedef CHRONO::nanoseconds                   duration;
      /** @brief duration storage type
       * 
       * The type used to count ticks by this clock
       */
      typedef duration::rep                                rep;
       /** @brief clock period
       * 
       * the period of this clock. 
       */
      typedef duration::period                             period;
      /** @brief Clock time point
       *
       * The type to represent a time point (or date) for this clock.
       */
      typedef CHRONO::time_point<dune_steady_clock> time_point;

      /** @brief Steadyness flag
       *
       * The flag that indicate if this clock is steady or not. 
       * A non steady clock do not guarantee that all the 
       * timepoints will be ordered (for example when the clock
       * time is reset to an anterior date).  
       */
      static const bool is_steady = true;
      
      /** @brief Current date
       *
       * @return the current time point
       */
      static time_point now();
      
    }; // TREX::LSTS::dune_steady_clock  

    /** @brief TREX dune steady clock
     *
     * The default TREX implemenation for the dune steady clock. 
     * The period of this clock is set to 1000Hz to allow some 
     * flexibility on the duration of a tick while not using to 
     * much precision (There's no need to run TREX at a tick rate
     * higher than 1000Hz  
     *
     * @relates dune_steady_clock
     * @sa posix_clock
     */
    typedef agent::rt_clock<CHRONO_NS::milli, dune_steady_clock> steady_clock;

  } // TREX::LSTS
} // TREX

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
    struct clock_string<TREX::LSTS::dune_posix_clock, CharT> {
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
    struct clock_string<TREX::LSTS::dune_steady_clock, CharT> {
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

#endif // H_DuneClock
