/* -*- C++ -*- */
/** @file "StepClock.hh"
 * @brief definition of a pseudo clock
 *
 * This files defines a pseudo-clock used to allow to play a mission
 * as fast as the system can.
 *
 * @author Conor McGann & Frederic Py <fpy@mbari.org>
 * @ingroup agent
 */
#ifndef H_StepClock
# define H_StepClock

# include "Clock.hh"
# include <trex/utils/LogManager.hh>
# include <trex/utils/StringExtract.hh>

namespace TREX {
  namespace agent {
    
    /** @brief Step based pseudo clock
     *
     * This class implements a TREX clock that advances based on
     * the number of steps used. This number of step can be modified
     * for each tick allowing to reproduce a mission with similar CPU
     * resource as it was during the previous run.
     *
     * @note As this clock is just based on counting steps it is not
     * real-time dependent and can go as fast as the host CPU allows
     * or be paused if needed.
     *
     * @author Conor McGann & Frederic Py <fpy@mbari.org>
     * @ingroup agent
     */
    class StepClock :public Clock {
    public:
      /** @brief Constructor
       * @param sleepSeconds how long should the process sleep
       * @param stepsPerTick default maximum number of steps per tick
       */
      StepClock(double sleepSeconds, unsigned int stepsPerTick);
      /** @brief XML parsing constructor
       * @param node A XML clock definition
       *
       * Create a new instance based on the constent of @a node.
       * The expected structure of node is 
       * @code
       * <StepClock steps="<number>"/>
       * @endcode
       *
       * where @c @<number@> is the default number of steps
       * per tick.
       * @note If the @c steps attribute is not given its value
       * is set to a default of 50
       *
       * @throw TREX::utils::bad_string_cast unable to parse @c steps attribute
       */
      StepClock(rapidxml::xml_node<> const &node);
      /** @brief Destructor */
      ~StepClock() {}

      /** @brief get current tick date
       * This method advances the number of steps used in current
       * tick by 1. If this number is greater or equal to the
       * maximum number of steps allowed for this tick, then the tick
       * date is incremented
       *
       * @return current tick date
       *
       * @sa setMaxSteps(unsigned int) const
       */
      TREX::transaction::TICK getNextTick();
      /** @brief Change maximum number of steps for this tick
       * @param nSteps New maximum number of steps
       * @pre nSteps is greater than 0
       * 
       * Change the maximum number of steps for current tick to
       * @a nSteps and reset the number of steps used to 0.
       */
      void setMaxSteps(unsigned int nSteps) const;

    private:
      /** @brief step value filter
       * @param stepsPerTick desired number of steps per tick
       * @retval @a stepsPerTick if this value is greater than 0
       * @retval 50 otherwise
       */
      static unsigned int selectStep(unsigned int stepsPerTick);
      /** @brief parse maximum number of steps
       * @param steps An XML attribute
       * @pre @a steps value is an integer
       *
       * @throw TREX::utils::bad_string_cast unable to parse @a steps value
       * @retval the integer value of @a steps if it is not @c NULL
       * and positive
       * @retval 50 otherwise
       */
      static unsigned int parseStep(rapidxml::xml_attribute<> *steps);

      /** @brief current tick date */
      TREX::transaction::TICK m_tick;
      /** @brief step counter inside a tick */
      mutable unsigned int m_currentStep;
      /** @brief maximum allowed number of steps for this tick */
      mutable unsigned int m_stepsPerTick;
      /** @brief default maximum allowed number of steps  */
      unsigned int const m_stepsPerTickDefault;
      /** @brief TREX log entry point */ 
      static TREX::utils::SingletonUse<TREX::utils::LogManager> m_log;
    }; // TREX::agent::StepClock

  } // TREX::agent 
} // TREX

#endif // H_StepClock
