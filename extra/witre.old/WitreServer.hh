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

# include <trex/utils/log_manager.hh>
#include <Wt/WApplication>
#include <Wt/WServer>
#include <boost/thread.hpp>
#include <trex/transaction/reactor.hh>
#include "WitreGraph.hh"


namespace TREX {
  namespace witre {

    class WitreApplication;
    class WitreReactor;
    class WitreGraph;

    class WitreServer :public TREX::transaction::reactor,
    public TREX::transaction::graph::timelines_listener {
    
    public:

      class Error :public TREX::utils::Exception {
            public:
                Error(std::string const &what) throw()
                    :TREX::utils::Exception(what) {}
                virtual ~Error() throw() {}
      };

      // WitreApplication const &app() const;
      // WitreApplication &app();

      typedef std::list<goal_id> timed_goal;

      Wt::WServer const &wt() const;
      Wt::WServer &wt();

      void connect(WitreApplication *client, const boost::function<void()>& function);
      void disconnect(WitreApplication *client);

      std::string extTimelinesName(int i) { return externalTimelines[i].str(); }
      int extTimelinesSize() { return externalTimelines.size(); };
      const std::queue<std::string> receiveObs() { return observations; };
      bool acceptsGoal(TREX::utils::symbol const &name) { return find_external(name)->accept_goals(); }
      time_t getTime_t() { 
        time_t now = (tick_to_time(current_tick())-boost::posix_time::from_time_t(0)).value.total_seconds();
        return now; 
      }
      std::string getDependencies(std::string name);
      const timed_goal& plan() { return planTokens; };

      TREX::transaction::goal_id clientGoalPost(boost::property_tree::ptree::value_type const &xml);
      TREX::transaction::goal_id clientGoalPost(TREX::transaction::Goal const &g);
      TREX::transaction::Goal getGoal(std::string obs, std::string prd);

      WitreServer(TREX::transaction::reactor::xml_arg_type arg);
      ~WitreServer();

      bool attached() const {
	return NULL!=m_entry;
      }
      WitreReactor &reactor() const {
	return *m_entry;
      }
    private:
      //Trex functions
      void handle_init();
      void handle_tick_start();
      void notify(TREX::transaction::Observation const &obs);
      bool synchronize();
      void new_plan_token(goal_id const &t);
      void cancelled_plan_token(goal_id const &t);
      //End of Trex functions

      struct Connection {
        Connection(const std::string& id, WitreApplication *c, const boost::function<void()>& f)
          : sessionId(id), client(c), function(f)
        { }

        std::string sessionId;
        WitreApplication *client;
        boost::function<void()> function;
      };

      void declared(TREX::transaction::details::timeline const &timeline);
      void undeclared(TREX::transaction::details::timeline const &timeline);

      void searchGraph();
      void dispatchPlanTokens();


      mutable boost::mutex mutex_;
      boost::thread thread_;
      std::vector<Connection> connections;
      std::set<utils::symbol> pendingTimelines;
      std::vector<utils::symbol> externalTimelines;
      std::queue<std::string> observations;
      timed_goal pastTokens;
      timed_goal planTokens;
      WitrePaintSearch::GraphMap timelineGraph;

      typedef TREX::utils::log::entry::date_type log_date;
      typedef TREX::utils::log::id_type          log_id;
      
      class log_proxy {
      public:
        typedef void result_type;
        typedef TREX::utils::log::entry::pointer argument_type;
        
        log_proxy(WitreServer &caller):me(caller) {}
        
        void operator()(argument_type msg);
        
      private:
        WitreServer &me;
      };      
     
      friend class log_proxy;
      boost::signals2::connection m_log_conn;

      bool attach(WitreReactor &r);
      void detach(WitreReactor &r);

      // UNIQ_PTR<WitreApplication> m_app;
      Wt::WServer *    m_server;

      TREX::utils::singleton::use<TREX::utils::log_manager> m_log;
      WitreReactor *m_entry;

      friend class TREX::utils::singleton::wrapper<WitreServer>;
      friend class WitreReactor;
    }; // TREX::witre::WitreServer

  } // TREX::witre
} // TREX

#endif // H_WitreServer
