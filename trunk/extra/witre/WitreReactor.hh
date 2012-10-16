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
#ifndef H_TREX_witre_reactor 
# define H_TREX_witre_reactor 

#include <trex/agent/Agent.hh>
#include <Wt/WSignal>


namespace TREX {
  namespace witre {
    
    class WitreReactor :public agent::Agent::AgentProxy,
    public transaction::graph::timelines_listener, 
    public Wt::WObject {
    public:
      typedef agent::Agent const *agent_ref;
      
      WitreReactor(agent::Agent &a);
      ~WitreReactor();

      Wt::Signal<agent_ref, transaction::TICK> &tick() {
        return m_tick;
      }
      Wt::Signal<agent_ref, utils::Symbol> &new_timeline() {
        return m_new_tl;
      }
      Wt::Signal<agent_ref, utils::Symbol> &failed_timeline() {
        return m_failed_tl;
      }
      Wt::Signal<agent_ref, transaction::Observation> &observation() {
        return m_obs;
      }
      
    private:
      agent_ref agent() const {
        return dynamic_cast<agent_ref>(&getGraph());
      }
      
      void handleTickStart();
      void declared(transaction::details::timeline const &timeline);
      void undeclared(transaction::details::timeline const &timeline);
      void notify(transaction::Observation const &obs);
      
      Wt::Signal<agent_ref, transaction::TICK> m_tick;
      Wt::Signal<agent_ref, utils::Symbol> m_new_tl;
      Wt::Signal<agent_ref, utils::Symbol> m_failed_tl;
      Wt::Signal<agent_ref, transaction::Observation> m_obs;
    }; // TREX::witre::WitreReactor
               
  } // TREX::witre
} // TREX

#endif // H_TREX_witre_reactor
