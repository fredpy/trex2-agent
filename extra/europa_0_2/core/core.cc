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
#include "Assembly.hh"
#include "DeliberationFilter.hh"
#include "ModeConstraints.hh"

#include "SynchronizationManager.hh"
#include "private/CurrentState.hh"

namespace TREX {
  namespace europa {
    
    /** @brief TREX core extensions for europa
     * 
     * This classe defines all the basic extensions of europa that
     * provide the core functionalities for the europa reactor. And are
     * used by T_REEX to implement synchronization and execution or for
     * the nddl modeler to acces to information that resides on T-REX in
     * the model. These extensions include :
     * @li @c DeliberationScope The filter that limits europa to plan only in
     * the planning window of the reactor 
     * [current tick, current tick+latency+lookahed]
     * @li @c SynchronizationScope The filter used during suynchronization
     * to limit the planning problem to only the current tick
     * @li CheckInternal A constraint that can be used to check if a
     * europa object is a TREX Internal timeline
     * @li CheckExternal A constraint that can be used to check if a
     * europa object is a TREX External timeline.
     *
     * @author Frederic Py <fpy@mbari.org>
     */
    class CoreExtensions :public EuropaPlugin {
    public:
      void registerComponents(Assembly const &assembly);
      
    }; // TREX::europa::CoreExtensions

    void CoreExtensions::registerComponents(Assembly const &assembly) {
      // T-REX core extensions
      TREX_REGISTER_FLAW_FILTER(assembly, TREX::europa::DeliberationScope, 
      				TREX_DELIB_FILT); 
      TREX_REGISTER_FLAW_FILTER(assembly, TREX::europa::SynchronizationScope, 
      				TREX_SYNCH_FILT); 
      TREX_REGISTER_FLAW_MANAGER(assembly, TREX::europa::SynchronizationManager,
				 TREX_SYNCH_MGR);
      TREX_REGISTER_FLAW_HANDLER(assembly, TREX::europa::details::CurrentState::DecisionPoint,
				 TREX_SYNCH_HANDLER);

      // The new flaw to be matched
      TREX_REGISTER_MATCH_FINDER(assembly, TREX::europa::details::UpdateMatchFinder, 
                                 TREX::europa::details::CurrentState::entityTypeName());    

      // T-REX constraints
      TREX_REGISTER_CONSTRAINT(assembly,TREX::europa::CheckExternal,
			       isExternal,trex);
      TREX_REGISTER_CONSTRAINT(assembly,TREX::europa::CheckInternal,
			       isInternal,trex); 

    }

    CoreExtensions trex_core;

  } // TREX::europa
} // TREX
