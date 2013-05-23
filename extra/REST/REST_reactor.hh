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
#ifndef H_trex_REST_reactor
# define H_trex_REST_reactor

# include <trex/transaction/TeleoReactor.hh>
# include "REST_service.hh"

# include <Wt/WServer>

# include <boost/thread.hpp>
# include <boost/signals2/signal.hpp>
# include <boost/operators.hpp>

# include <Wt/Dbo/backend/Sqlite3>
# include <Wt/Dbo/Dbo>



namespace TREX {
  namespace REST {
    
    class REST_reactor;
    
    namespace bits {
      class tick_wait;
      
      class timeline_info
      :boost::less_than_comparable1<timeline_info,
      boost::less_than_comparable2<timeline_info, TREX::utils::Symbol> > {
      public:
        timeline_info(transaction::details::timeline const &tl, REST_reactor &r)
        :m_timeline(tl), m_reactor(&r) {}
        ~timeline_info() {}
        
        boost::property_tree::ptree basic_tree(bool complete=true) const;
        utils::Symbol const &name() const;
        
        bool alive() const;
        bool accept_goals() const;
        
        bool operator< (TREX::utils::Symbol const &name) const;
        bool operator< (timeline_info const &other) const;
        
        void notify(transaction::Observation const &obs, transaction::TICK cur) const;
        void future(transaction::IntegerDomain const &f) const;
        
        boost::property_tree::ptree list_tokens(transaction::IntegerDomain const &range) const;
        
      private:
        
        boost::property_tree::ptree duration_tree(transaction::TICK d) const;
        boost::property_tree::ptree token_tree(transaction::goal_id g) const;
        
        transaction::details::timeline const &m_timeline;
        mutable transaction::TICK                     m_last_tick;
        mutable transaction::goal_id                  m_last;
        REST_reactor *m_reactor;
      };
      
    }
    
    class REST_reactor: public TREX::transaction::TeleoReactor,
    public TREX::transaction::graph::timelines_listener {
    public:
      REST_reactor(TREX::transaction::TeleoReactor::xml_arg_type arg);
      ~REST_reactor();
      
    private:
      
      
      //reactor handlers
      void handleInit();
      void handleTickStart();
      void notify(TREX::transaction::Observation const &obs);
      bool synchronize();
      void newPlanToken(transaction::goal_id const &t);
      void cancelledPlanToken(transaction::goal_id const &t);

      // timelines events
      void declared(transaction::details::timeline const &timeline);
      void undeclared(transaction::details::timeline const &timeline);
      
      boost::property_tree::ptree tick_info(transaction::TICK date) const;
      boost::property_tree::ptree tick_period(rest_request const &req) const;
      boost::property_tree::ptree get_tick(rest_request const &req) const;
      boost::property_tree::ptree tick_at(rest_request const &req) const;
      boost::property_tree::ptree next_tick(rest_request const &) const {
        return tick_info(getCurrentTick()+1);
      }
      boost::property_tree::ptree initial_tick(rest_request const &) const {
        return tick_info(getInitialTick());
      }
      boost::property_tree::ptree final_tick(rest_request const &) const {
        return tick_info(getFinalTick());
      }
      boost::property_tree::ptree wait_tick(rest_request &req);
      
      boost::property_tree::ptree trex_version(rest_request const &req) const;

      boost::property_tree::ptree timelines(rest_request const &req);
      boost::property_tree::ptree timeline(rest_request const &req);
      
      boost::property_tree::ptree export_goal(transaction::goal_id g) const;
      
      boost::property_tree::ptree goals(rest_request const &req);
      boost::property_tree::ptree manage_goal(rest_request const &req);

      boost::property_tree::ptree get_timeline(std::string name,
                                               transaction::IntegerDomain range);
      
      void add_goal(transaction::goal_id g);
      transaction::goal_id get_goal(std::string const &id) const;
      bool remove_goal(std::string const &id);
      
      boost::property_tree::ptree list_goals(rest_request const &req) const;
      
      void add_tl(bits::timeline_info const &tl);
      void remove_tl(utils::Symbol const &tl);
      
      void add_obs(transaction::Observation obs, transaction::TICK cur);
      void extend_obs(transaction::TICK cur);
      
      template<typename Ret>
      Ret strand_run(boost::function<Ret ()> const &f) {
        boost::packaged_task<Ret> tsk(f);
        boost::unique_future<Ret> result = tsk.get_future();
        
        m_strand->post(boost::bind(&boost::packaged_task<Ret>::operator(),
                                  boost::ref(tsk)));
        return result.get();
      }
      
      
      size_t get_id();
      
      typedef boost::signals2::signal<void (transaction::TICK)> tick_event;
      tick_event m_tick_signal;
      
      UNIQ_PTR<Wt::WServer>           m_server;
      UNIQ_PTR<service_tree>          m_services;

      UNIQ_PTR<boost::asio::strand>   m_strand;
      
      // Observation database
      Wt::Dbo::Session                    m_obs_session;
      UNIQ_PTR<Wt::Dbo::backend::Sqlite3> m_obs_db;
      
      typedef std::set<bits::timeline_info> tl_set;
      tl_set m_timelines;
      
      
      typedef std::map<std::string, transaction::goal_id> goal_map;
      
      goal_map m_goals;
      
      utils::SharedVar<size_t> m_file_count;
      
      friend class bits::tick_wait;
      friend class bits::timeline_info;
    };
    
    template<>
    inline void REST_reactor::strand_run<void>(boost::function<void ()> const &f) {
      boost::packaged_task<void> tsk(f);
      boost::unique_future<void> result = tsk.get_future();
      
      m_strand->post(boost::bind(&boost::packaged_task<void>::operator(),
                                 boost::ref(tsk)));
      result.get();
    }

    
  }
}

#endif // H_trex_REST_reactor
