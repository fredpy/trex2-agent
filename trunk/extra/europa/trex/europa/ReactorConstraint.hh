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
#ifndef H_trex_europa_ReactorConstraint
# define H_trex_europa_ReactorConstraint

# include "ReactorPropagator.hh"
# include "EuropaException.hh"

# pragma warning (push : 0)
// europa has a lot of warnings: lets make it more silent
#  include <PLASMA/Constraint.hh>
# pragma warning (pop)

namespace TREX {
  namespace europa {
    
    /** @brief Reactor information constraints
     *
     * This abstract class allows to implement a constraint that have access
     * to all the public methods of the Assembly it is tied too. By doing so one 
     * can implement constraints in europa that can manipulate information of
     * the reactor within the T-REX agent or the agent itself.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup europa
     */
    class ReactorConstraint :public EUROPA::Constraint {
    public:
      /** @brief Constructor
       *
       * @param[in] name symbolic name of the constraint
       * @param[in] propagatorName symbolic name of the propagator that handle this constraint
       * @param[in] cstrEngine A constraint engine
       * @param[in] vars The list of argument passed as argument to this constraint
       *
       * The standard signature for a ceuropa constraint constructor as expected from 
       * the constraint factory from europa
       *
       * @pre The propagator @p propagatorName should be a ReactorPropagator. Every Assembly instance 
       * often create the propagator @c "trex" which is of this type
       *
       * @sa ReactorPropagator
       */
      ReactorConstraint(EUROPA::LabelStr const &name,
			EUROPA::LabelStr const &propagatorName,
			EUROPA::ConstraintEngineId const &cstrEngine,
			std::vector<EUROPA::ConstrainedVariableId> const &vars);
      /** @brief Destructor */
      virtual ~ReactorConstraint() {}
      
    protected:
      /** @brief Get the attached assembly
       *
       * @pre this constraint instance was created by an Assembly
       * @throw EuropaException The propagator for this constraint is not a ReactorPropagator
       * @return the Assembly that manipulate this constraint
       */
      Assembly &assembly();
      
    private:
      Assembly *m_assembly;
    }; // TREX::europa::ReactorConstraint

  } // TREX::europa
} // TREX

#endif // H_trex_europa_ReactorConstraint
