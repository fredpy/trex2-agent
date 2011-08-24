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
#ifndef H_LogPlayer
# define H_LogPlayer

# include "TeleoReactor.hh"

namespace TREX {
  namespace transaction {

    class LogPlayer :public TeleoReactor {
    public:
      LogPlayer(TeleoReactor::xml_arg_type arg);
      ~LogPlayer();

    private:

      void handleInit();
      void handleTickStart();
      bool synchronize();

      void loadTransactions(rapidxml::xml_node<> &root, std::string const &reactor);
      
      std::multimap<TICK, Observation> m_obsLog;
      std::multimap<TICK, Observation>::const_iterator m_nextObs;

      std::multimap<TICK, goal_id> m_requestLog;
      std::multimap<TICK, goal_id>::const_iterator m_nextRequest;
      std::multimap<TICK, goal_id> m_recallLog;
      std::multimap<TICK, goal_id>::const_iterator m_nextRecall;
      
    }; // TREX::transaction::LogPlayer

  } // TREX::transaction 
} // TREX 

#endif // H_LogPlayer
