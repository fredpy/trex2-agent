/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2012, MBARI.
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
#ifndef H_trex_europa_Numeric
# define H_trex_europa_Numeric

# include <trex/europa/config.hh>

# include <PLASMA/Constraint.hh>

namespace TREX {
  namespace europa {

    /** @brief Absolute Value Constraint
     *
     * This class implements the absolute value constraint within europa
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup europa
     */
    class AbsValConstraint :public EUROPA::Constraint {
    public:
      /** @brief Constructor
       * @param[in] name the name of the constraint
       * @param[in] propagatorName The name of the propagator handling this constraint
       * @param[in] cstrEngine the constraint engine for this instance
       * @param[in] vars the list of arguments
       *
       * @pre @p vars size is @e exactly 2
       * @pre All the element of @p vars are IntervalDomain instances
       */
     AbsValConstraint(EUROPA::LabelStr const &name,
		      EUROPA::LabelStr const &propagatorName,
		      EUROPA::ConstraintEngineId const &cstrEngine,
		      std::vector<EUROPA::ConstrainedVariableId> const &vars);
    private:
      void handleExecute();
      
      EUROPA::Domain &m_abs;
      EUROPA::Domain &m_val;
    }; // TREX::europa::AbsValConstraint

    /** @brief Square root Constraint
     *
     * This class implements the square root constraint within europa
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup europa
     */
    class SqrtConstraint :public EUROPA::Constraint {
    public:
      /** @brief Constructor
       * @param[in] name the name of the constraint
       * @param[in] propagatorName The name of the propagator handling this constraint
       * @param[in] cstrEngine the constraint engine for this instance
       * @param[in] vars the list of arguments
       *
       * @pre @p vars size is @e exactly 2
       * @pre All the element of @p vars are IntervalDomain instances
       */
      SqrtConstraint(EUROPA::LabelStr const &name,
                     EUROPA::LabelStr const &propagatorName,
                     EUROPA::ConstraintEngineId const &cstrEngine,
                     std::vector<EUROPA::ConstrainedVariableId> const &vars);
    private:
      void handleExecute();
      
      EUROPA::Domain &m_sqrt;
      EUROPA::Domain &m_val;
    }; // TREX::europa::SqrtConstraint
    
    
  } // TREX::europa
} // TREX

#endif // H_trex_europa_Numeric
