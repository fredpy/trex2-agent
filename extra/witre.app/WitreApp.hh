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
#ifndef H_TREX_witre_WitreApp
# define H_TREX_witre_WitreApp

# include "TREXLogWidget.hh"
# include "WitreServer.hh"

# include <Wt/WApplication>
# include <Wt/WPopupMenu>

namespace TREX {
  namespace witre {

    class WitreApp :public Wt::WApplication {
    public:
      WitreApp(Wt::WEnvironment const &env, Server &server);
      ~WitreApp();
      
      void agent_updated();
      void log_updated();
      
    private:
      void about();
      void updateAgentState();
      
      void start_agent();
      void stop_agent();
      
      void ping();
      
      void process_updates();
      
      Server &m_server;
      TREXLogWidget      *m_log;
      Wt::WPopupMenu     m_menu;
      Wt::WPopupMenuItem *m_start;
      Wt::WPopupMenuItem *m_stop;
      
      Wt::WTimer *m_refresh;
      
      bool   m_agent_updated;
      size_t m_log_count;
    }; // TREX::witre::WitreApp

  }
}

#endif
