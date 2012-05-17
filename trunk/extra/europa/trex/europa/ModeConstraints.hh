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
#ifndef H_trex_europa_ModeConstraints
# define H_trex_europa_ModeConstraints

# include "ReactorConstraint.hh"

namespace TREX {
  namespace europa {
    
    /** @brief Check if Internal timeline
     *
     * This europa constraint check if a NDDL object is an Internal timeline 
     * of the associated reactor.
     * 
     * This constraint is usually used within a NDDL rule and support the 
     * following syntaxes:
     * @li @c isInternal(bool, obj) where @c bool domain is restricted whenever 
     *    all the possible values of @c obj are either necessarily @e Internal 
     *    timelines or necessarily not
     * @li @c isInternal(bool) behaving as above but the object is the one for 
     *    which the rule apply
     *
     * We also support the functional form of these constraints which return the 
     * boolean: 
     * @li @c isInternal(obj)
     * @li @c isInternal()
     * @note While one vcould directly test the @c mode of an @c AgentTimeline, 
     * using this @c isInternal constraints is highly recommended as it asks 
     * directly to the reactor the mode of the timline and is therefore robust 
     * to timeline promotion/demotion from/to @e External when the timeline was 
     * already owned by another reactor
     *
     * @sa CheckExternal
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup europa
     */
    class CheckInternal :public ReactorConstraint {
    public:
      CheckInternal(EUROPA::LabelStr const &name, 
		    EUROPA::LabelStr const &propagatorName,
		    EUROPA::ConstraintEngineId const &cstrEngine,
		    std::vector<EUROPA::ConstrainedVariableId> const &variables);
      void handleExecute();
    private:
      EUROPA::Domain *m_test;
      EUROPA::Domain *m_obj;
    }; // TREX::europa::CheckInternal

    /** @brief Check if External timeline
     *
     * This europa constraint check if a NDDL object is an External timeline 
     * of the associated reactor.
     * 
     * This constraint is usually used within a NDDL rule and support the 
     * following syntaxes:
     * @li @c isExternal(bool, obj) where @c bool domain is restricted whenever 
     *    all the possible values of @c obj are either necessarily @e External 
     *    timelines or necessarily not
     * @li @c isExternal(bool) behaving as above but the object is the one for 
     *    which the rule apply
     *
     * We also support the functional form of these constraints which return the 
     * boolean: 
     * @li @c isExternal(obj)
     * @li @c isExternal()
     * @note While one vcould directly test the @c mode of an @c AgentTimeline, 
     * using this @c isInternal constraints is highly recommended as it asks 
     * directly to the reactor the mode of the timline and is therefore robust 
     * to timeline demotion/promotion from/to @e Internal when the timeline was 
     * already owned by another reactor
     *
     * @sa CheckInternal
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup europa
     */
    class CheckExternal :public ReactorConstraint {
    public:
      CheckExternal(EUROPA::LabelStr const &name, 
		    EUROPA::LabelStr const &propagatorName,
		    EUROPA::ConstraintEngineId const &cstrEngine,
		    std::vector<EUROPA::ConstrainedVariableId> const &variables);
      void handleExecute();
    private:
      EUROPA::Domain *m_test;
      EUROPA::Domain *m_obj;
    }; // TREX::europa::CheckInternal

  } // TREX::europa
} // TREX

#endif // H_trex_europa_ModeConstraints
