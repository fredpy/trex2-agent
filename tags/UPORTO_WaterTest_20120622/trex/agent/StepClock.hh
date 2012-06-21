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
/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2011, MBARI.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TREX Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
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
      StepClock(duration_type const &slp, 
                unsigned int stepsPerTick);
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
      StepClock(boost::property_tree::ptree::value_type &node);
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
      std::string info() const {
        std::ostringstream oss;
        oss<<"Simulated clock with "<<m_stepsPerTickDefault
          <<" deliberation steps per tick.";
        return oss.str();
      }

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
      static unsigned int parseStep(boost::property_tree::ptree &steps);

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
