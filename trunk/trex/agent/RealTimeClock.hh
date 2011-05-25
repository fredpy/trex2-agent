/* -*- C++ -*- */
/** @file "RealTimeClock.hh"
 * @brief definition of a system based real-time clock for TREX
 *
 * This files defines a real time clock which is the default clock
 * for the TREX agent
 *
 * @author Conor McGann & Frederic Py <fpy@mbari.org>
 * @ingroup agent
 */
#ifndef H_RealTimeClock
# define H_RealTimeClock

# include <boost/thread/recursive_mutex.hpp>

# include "Clock.hh"
# include <trex/utils/TimeUtils.hh>
# include <trex/utils/LogManager.hh>
# include <trex/utils/StringExtract.hh>

namespace TREX {
  namespace agent {
    
    /** @brief Real time default clock for TREX
     *
     * This class implements a real-time clock based on @c gettimeofday
     * primitive to be used for TREX agent.
     *
     * @author Conor McGann & Frederic Py <fpy@mbari.org>
     * @ingroup agent
     */
    class RealTimeClock :public Clock {
    public: 
      /** @brief Constructor
       * @param secondsPerTick tick duration in seconds
       *
       * Create a new instance with the tick duration set to
       * @a secondsPerTick
       *
       * @pre @a secondsPerTick should be greater than 0.0
       */
      explicit RealTimeClock(double secondsPerTick);
      /** @brief XML parsing constructor
       * @param node A XML clock definition
       *
       * Create a new instance based on the constent of @a node.
       * The expected structure of node is 
       * @code
       * <RealTime tick="<duration>"/>
       * @endcode
       *
       * where @c @<duration@> is a float giving the number
       * of seconds per tick.
       *
       * @throw TREX::utils::Exception @c tick attribute missing
       * @throw TREX::utils::bad_string_cast unable to parse tick attribute
       * @throw TREX::utils::Exception @c tick attribute value is less than 0
       */
      explicit RealTimeClock(rapidxml::xml_node<> const &node);
      /** @brief Destructor */
      ~RealTimeClock() {}

      /** @brief Compute tick date
       *
       * This method get the system time and identify what is the current tick
       *
       * @return current time in tick
       *
       * @note If more than one tick duration happened between the last
       * call and this one. This method will display it in the TREX log.
       */
      TREX::transaction::TICK getNextTick();

      double tickDuration() const {
	return m_floatTick;
      }


    private:
      /** @brief Start the clock
       * initialize the clock based on current system time
       */
      void start();
      /** @brief get time left
       * @return time left before next tick in seconds
       * @note if the time left is negative it will display a message
       * in TREX log and return 0
       */
      double getSleepDelay() const;
      
      /** @brief get real-time date
       * @param val output variable
       * Gets system time and store it in @a val
       */
      static void getDate(timeval &val);
      /** @brief Compute next deadline
       *
       * @param factor tick increment
       *
       * This method increments the deadline by @a factor multiplied
       * by the tick duration.
       *
       * @note @a factor is used in case the clock missed several ticks
       */
      void setNextTickDate(unsigned factor=1);
      /** @brief time left before next tick
       * @return number of seconds left before the next tick
       *
       * @sa double getSleepDelay() const
       */
      double timeLeft() const;

      /** @brief Clock status flag
       * Indicates whether the clack has been started or not
       */
      bool m_started;
      /** @brief Current tick date */
      TREX::transaction::TICK m_tick;
      /** @brief Tick duration as a @c float
       * @sa m_secondsPerTick
       */
      double m_floatTick;
      /** @brief Tick duration as a @c timeval
       * @sa m_floatTick
       */
      timeval m_secondsPerTick;
      /** @brief Tick deadline */
      timeval m_nextTickDate;
      
      typedef boost::recursive_mutex mutex_type;

      /** @brief mutex for basic clock calls */
      mutable mutex_type m_lock;
      /** @brief TREX log entry point */
      TREX::utils::SingletonUse<TREX::utils::LogManager> m_log;
      
    }; // TREX::agent::RealTimeClock

  } // TREX::agent 
} // TREX

#endif // H_RealTimeClock
