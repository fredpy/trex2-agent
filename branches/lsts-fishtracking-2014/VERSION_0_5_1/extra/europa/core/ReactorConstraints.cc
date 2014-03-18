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
#include "trex/europa/ReactorConstraint.hh"
#include "trex/europa/bits/europa_helpers.hh"

using namespace TREX::europa;

/*
 * class TREX::europa::ReactorPropagator
 */

/*
 * class TREX::europa::ReactorConstraint
 */
// structors 

ReactorConstraint::ReactorConstraint(EUROPA::LabelStr const &name,
				     EUROPA::LabelStr const &propagatorName,
				     EUROPA::ConstraintEngineId const &cstrEngine,
				     std::vector<EUROPA::ConstrainedVariableId> const &vars)
  :EUROPA::Constraint(name, propagatorName, cstrEngine, vars), m_assembly(NULL) {}


// observers 

Assembly &ReactorConstraint::assembly() {
  if( NULL==m_assembly ) {
    EUROPA::PropagatorId prop = getPropagator();

    if( EUROPA::Id<ReactorPropagator>::convertable(prop) ) {
      EUROPA::Id<ReactorPropagator> r_prop(prop);
      m_assembly = &(r_prop->m_assembly);
    } else {
      throw EuropaException("Constraint "+getName().toString()+" is attached to "+
			    prop->getName().toString()+
			    " which is not a reactor propagator.");
    }
  }
  return *m_assembly;
}
