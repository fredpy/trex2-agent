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
#ifndef H_trex_europa_TimeConstraints
# define H_trex_europa_TimeConstraints

# include "ReactorConstraint.hh"

namespace TREX {
  namespace europa {

    /** @brief date to tick constraint
     *
     * This class implements an europa constrainty that allow to convert a
     * "real-time" date into a europa tick date or converselly.
     * The conversion is handled through the clock used by the current 
     * clock used by the agent.
     *
     * Such comstraint is pecifically usefull when one want to specify 
     * to europa a real date (in unix time format) and have it used 
     * during planning which the require to adapt this date into the 
     * tick format used.
     *
     * @sa Assembly::date_to_tick
     * @sa Assembly::tick_to_date
     *
     * It is provided as a nddl constraint with following format
     * @code 
     *  tick_date(tick, date)
     * @endcode
     * where @c tick and @c date are two integer variables
     *
     * @ingroup europa
     * @author Frederic Py <fpy@mbari.org>
     */
    class TickFromDate :public ReactorConstraint {
    public:
      /** @brief Constructor
       *
       * @param[in] name The name of the constraint in the model
       * @param[in] propagtorName Name of the constraint propagator used
       * @param[in] cstrEngine Reference to the constraint engine
       * @param[in] variables The variables given as argument
       *
       * @pre @p variables size is 2
       * @pre @p variables contains only integer varaiables or equivalent
       */
      TickFromDate(EUROPA::LabelStr const &name, 
		 EUROPA::LabelStr const &propagatorName,
		 EUROPA::ConstraintEngineId const &cstrEngine,
		 std::vector<EUROPA::ConstrainedVariableId> const &variables);
      /** @brief Execution handler
       *
       * Propagate the constraint. It applies the conversion from tick 
       * to date to the variables given at construction. The propgation 
       * is bidirectional meaning that it restrict both the domain of 
       * the two variables based on the current variable of their
       * counterpart. 
       */
      void handleExecute();
    private:
      EUROPA::Domain *m_tick;
      EUROPA::Domain *m_date;

      enum indexes {
	TICK =0,
	DATE =1,
	NARGS = 2
      };
    };

  } // TREX::europa
} // TREX

#endif // H_trex_europa_TimeConstraints
