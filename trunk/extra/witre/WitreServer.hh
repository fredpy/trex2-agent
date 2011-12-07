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
#ifndef H_WitreServer
# define H_WitreServer

# include <trex/utils/LogManager.hh>
#include <Wt/WApplication>
#include <Wt/WServer>
#include <boost/thread.hpp>
#include <trex/transaction/TeleoReactor.hh>
#include "Observations.hh"


namespace TREX {
  namespace witre {

    class WitreApplication;
    class WitreReactor;

    class WitreServer :public TREX::transaction::TeleoReactor, boost::noncopyable {
    public:

      class Error :public TREX::utils::Exception {
      public:
	Error(std::string const &what) throw()
	  :TREX::utils::Exception(what) {}
	virtual ~Error() throw() {}
      };

      // WitreApplication const &app() const;
      // WitreApplication &app();

      Wt::WServer const &wt() const;
      Wt::WServer &wt();

      void connect(WitreApplication *client, const boost::function<void()>& function);
      void disconnect(WitreApplication *client);

      std::string extTimelinesName(int i) { return externalTimelines[i].str(); }
      int extTimelinesSize() { return externalTimelines.size(); };
      const std::queue<Observations> receiveObs() { return observations; };
      bool acceptsGoal(TREX::utils::Symbol const &name) { return find_external(name)->accept_goals(); }
      time_t getTime_t() { time_t now = std::floor(tickToTime(getCurrentTick())); return now; };

      TREX::transaction::goal_id clientGoalPost(TREX::transaction::Goal const &g);
      TREX::transaction::Goal getGoal(std::string obs, std::string prd);

      WitreServer(TREX::transaction::TeleoReactor::xml_arg_type arg);
      ~WitreServer();

      bool attached() const {
	return NULL!=m_entry;
      }
      WitreReactor &reactor() const {
	return *m_entry;
      }
    private:
      //Trex functions
      void handleInit();
      void handTickStart() {};
      void notify(TREX::transaction::Observation const &obs);
      bool synchronize();
      //End of Trex functions

      struct Connection {
        Connection(const std::string& id, WitreApplication *c, const boost::function<void()>& f)
          : sessionId(id), client(c), function(f)
        { }

        std::string sessionId;
        WitreApplication *client;
        boost::function<void()> function;
      };

      mutable boost::mutex mutex_;
      boost::thread thread_;
      std::vector<Connection> connections;
      std::vector<utils::Symbol> externalTimelines;
      std::queue<Observations> observations;


      bool attach(WitreReactor &r);
      void detach(WitreReactor &r);

      // std::auto_ptr<WitreApplication> m_app;
      Wt::WServer *    m_server;

      TREX::utils::SingletonUse<TREX::utils::LogManager> m_log;
      WitreReactor *m_entry;

      friend class TREX::utils::SingletonWrapper<WitreServer>;
      friend class WitreReactor;
    }; // TREX::witre::WitreServer

  } // TREX::witre
} // TREX

#endif // H_WitreServer
