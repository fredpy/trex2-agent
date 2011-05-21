/* -*- C++ -*- */
/** @file Clock.hh
 * @brief TREX Clock
 *
 * This class defines the basic utilities for time representation
 * and manipulation inside TREX.
 *
 * @author Conor McGann & Frederic Py <fpy@mbari.org>
 * @ingroup transaction
 */
#ifndef H_Clock
#define H_Clock

# include "Tick.hh"
# include "ErrnoExcept.hh"
# include "XmlFactory.hh"

namespace TREX {
  namespace agent {

    /** @brief TREX clock
     *
     * This class defines the abstract interface to the Clock that
     * manage the TICK advance.
     *
     * @author Conor McGann
     * @ingroup agent
     */
    class Clock {
    public:
      /** @brief Destructor */
      virtual ~Clock() {}

      /** @brief Start the clock
       *
       * This method is the public interface used to start
       * the clock.
       *
       * @sa void start()
       */
      void doStart();

      /** @brief get time in TICK
       *
       * This method is used to compute the current time.
       * This is where one specialization of clock will identify
       * whether the TICK value has advanced or not
       *
       * @return current tick value
       */
      virtual TREX::transaction::TICK getNextTick() =0;

      virtual TREX::transaction::TICK initialTick() const {
	return 0;
      }
      /** @brief Sleep until next tick
       *
       * This method check the time left before next tick and
       * put the process asleep for this duration.
       *
       * @throw ErrnoExcept An error occurred while trying to sleep
       *
       * @sa double getSleepDelay() const
       * @sa void sleep(double)
       */
      virtual void sleep() const;
      /** @brief Process sleep
       * @param sleepDuration number of seconds to sleep
       *
       * This methods make the calling process to sleep for
       * @a sleepDuration seconds
       *
       * @throw ErrnoExcept An error occurred while trying to sleep
       */
      static void sleep(double sleepDuration);

      /** @brief XML factory for clocks. */
      typedef TREX::utils::XmlFactory<Clock> xml_factory;
      
    protected:
      /** @brief Time left before next tick
       *
       * This method indicates how much time is left before the
       * next tick
       *
       * @return time left before next tick
       */
      virtual double getSleepDelay() const {
	return m_sleepSeconds;
      }
      /** @brief Constructor
       * @param sleepSeconds A default sleep delay
       * Create a new instance with the default sleeping delay set to
       * @a sleepSeconds
       */
      explicit Clock(double sleepSeconds)
	:m_sleepSeconds(sleepSeconds) {}

      /** @brief Internal start method
       *
       * This method is called internally by @c doStart to allow specialized
       * initialization before starting the clock.
       *
       * @sa void doStart()
       */
      virtual void start() {}
      /** @brief increment tick value
       * @param tick A tick variable
       *
       * increase the value of @a tick by 1
       *
       * @deprecated This method was used for computing extra stats.
       * We need to find a better approach. Specifically it would be
       * relevant to have the tick variable located on this class.
       */
      void advanceTick(TREX::transaction::TICK &tick);

    private:
      double const m_sleepSeconds;

    }; // TREX::agent::Clock

  } // TREX::agent   
} // TREX

#endif // H_Clock
