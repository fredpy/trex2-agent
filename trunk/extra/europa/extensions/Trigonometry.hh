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
#ifndef H_trex_europa_Trigonometry
# define H_trex_europa_Trigonometry

# include <trex/europa/config.hh>

# include <PLASMA/Constraint.hh>

namespace TREX {
  namespace europa {

    /** @brief Cosine constraint 
     * 
     * A constraint that compute the cosine of an angle in degree.
     * The first argument is an angle domain and the second will store the 
     * resulting cosine value. 
     * 
     * @note As of today we do not restrict the angle based on the cosine 
     *       domain even when the angle domain span would alow it. Nonetheless 
     *       the domain of the cosine will be restricted even though the angle 
     *       is not a singleton (ie cos([0 90], var) will restrict var to [0 1])  
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup europa
     */
    class CosineConstraint :public EUROPA::Constraint {
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
      CosineConstraint(EUROPA::LabelStr const &name,
			EUROPA::LabelStr const &propagatorName,
			EUROPA::ConstraintEngineId const &cstrEngine,
			std::vector<EUROPA::ConstrainedVariableId> const &vars);
    private:
      /** @brief Execute the constraint
       *
       * Recompute the cosine of the angle and restrict the second argument 
       * accordingly
       */
      void handleExecute();
      
      EUROPA::Domain &m_angle;
      EUROPA::Domain &m_cos;
    }; // TREX::europa::CosineConstraint

    /** @brief Sine constraint 
     * 
     * A constraint that compute the sine of an angle in degree.
     * The first argument is an angle domain and the second will store the 
     * resulting sine value. 
     * 
     * @note As of today we do not restrict the angle based on the sine 
     *       domain even when the angle domain span would alow it. Nonetheless 
     *       the domain of the sine will be restricted even though the angle 
     *       is not a singleton (ie sin([0 90], var) will restrict var to [0 1])  
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup europa
     */
    class SineConstraint :public EUROPA::Constraint {
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
      SineConstraint(EUROPA::LabelStr const &name,
		     EUROPA::LabelStr const &propagatorName,
		     EUROPA::ConstraintEngineId const &cstrEngine,
		     std::vector<EUROPA::ConstrainedVariableId> const &vars);
    private:
      /** @brief Execute the constraint
       *
       * Recompute the sine of the angle and restrict the second argument 
       * accordingly
       */
      void handleExecute();
      
      EUROPA::Domain &m_angle;
      EUROPA::Domain &m_sin;
    }; // TREX::europa::SineConstraint
    
  } // TREX::europa
} // TREX

#endif // H_trex_europa_Trigonometry
