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
#ifndef H_AgentLocation
#define H_AgentLocation

#include <trex/transaction/TeleoReactor.hh>

namespace TREX {
  /** @brief AgentLocation plug-in utilities
   *
   * This namespace embeds the classes and functions provided by the
   * AgentLocation plug-in
   *
   * @ingroup AgentLocation
   *
   */
  namespace AgentLocation {

    class AgentLocation :public TREX::transaction::TeleoReactor {
    public:
      /** @brief XML constructor
       * @param arg An XML node definition
       *
       * This constructor is called to generate a location reactor
       * based on an XML node. The expected XML format is the following:
       * @code
       * <AgentLocation name="<name>" latency="<int>" lookahead="<int>" />
       * @endcode
       */
      AgentLocation(TREX::transaction::TeleoReactor::xml_arg_type arg);
      /** @brief Destructor */
      ~AgentLocation();

    private:
      void handleInit();
      bool synchronize();
      void handleRequest(TREX::transaction::goal_id const &g);
      void handleRecall(TREX::transaction::goal_id const &g);

      /** @brief State of the timeline */
      TREX::transaction::TICK m_nextSwitch;

      void setAt(std::string location);
      void setGo(std::string origin, std::string destination);

      std::list<TREX::transaction::goal_id> m_pending;
      std::auto_ptr<TREX::transaction::Observation> m_state;

      /** @brief Name of the predicate At */
      static TREX::utils::Symbol const AtPred;
      /** @brief Name of the predicate Go */
      static TREX::utils::Symbol const GoPred;

      /** @brief Name of the Timeline AgentLocation */
      static TREX::utils::Symbol const AgentLocationObj;

    };

  }
}

#endif // H_AgentLocation
