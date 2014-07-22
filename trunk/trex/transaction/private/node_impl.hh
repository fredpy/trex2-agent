/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, MBARI.
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
#ifndef H_trex_transaction_node_impl
# define H_trex_transaction_node_impl

# include "../bits/transaction_fwd.hh"
# include "../bits/reactor_policies.hh"

# include "external_handler.hh"

# include <trex/domain/token.hh>

# include <trex/utils/symbol.hh>
# include <trex/utils/log_manager.hh>

namespace TREX {
  namespace transaction {
    
    class reactor;
    
    namespace details {

      class node_impl :boost::noncopyable,
      public ENABLE_SHARED_FROM_THIS<node_impl> {
      public:
        node_impl(WEAK_PTR<graph_impl> const &g,
                  utils::symbol const &name,
                  exec_ref const &e);
        ~node_impl();
        
        void attach(SHARED_PTR<reactor> r);
        
        utils::symbol const &name() const {
          return m_name;
        }
        date_type latency()              const;
        date_type lookahead()            const;
        boost::optional<date_type> now()        const;
        boost::optional<date_type> last_synch() const;
        
        void set_latency(date_type const &val);
        void set_lookahead(date_type const &val);
        
        bool is_internal(utils::symbol const &tl) const;
        token_ref obs(utils::symbol const &tl,
                      utils::symbol const &pred) const;
        void post(token_ref obs);
        
        bool is_external(utils::symbol const &tl) const;

        void provide(utils::symbol const &tl, bool g, bool p);
        void use(utils::symbol const &tl, bool g, bool p);

        
        utils::log::stream syslog(utils::symbol const &who,
                                  utils::symbol const &kind) const;
 
        void isolate();
        void g_strand_notify(date_type tick, token_id const &obs,
                             bool fresh);        
      private:
        
        bool root() const;
        
        typedef std::set<SHARED_PTR<internal_impl>, tl_cmp> internal_set;
       
        friend class external_handler;

        external_handler m_externals;

        enum reactor_priorities {
          tick_event,
          obs_event,
          synch_event
        };
        
        bool dead() const;
        void kill();
        bool start();
        
        utils::singleton::use<utils::log_manager> m_log;
        
        WEAK_PTR<graph_impl> m_graph;
        WEAK_PTR<reactor>    m_reactor;
        utils::symbol const  m_name;
        exec_ref             m_exec;
        
        mutable boost::shared_mutex m_mtx;
        bool                        m_dead;
        date_type                   m_latency;
        date_type                   m_lookahead;
        internal_set                m_internals;
        
        // All methods prefixed by g_strand MUST be executed
        // by m_graph.lock()->strand()
        void g_strand_attach(SHARED_PTR<reactor> r);
        void g_strand_latency(date_type val);
        void g_strand_lookahead(date_type val);
        void g_strand_tick(boost::signals2::connection const &c,
                           date_type tick);
        void g_strand_isolate();
        void g_strand_synchronized(date_type date);
        void g_strand_declare(utils::symbol tl,
                              transaction_flags gp);
        void g_strand_use(utils::symbol tl,
                          transaction_flags gp);
        void g_strand_lost(SHARED_PTR<internal_impl> tl);
        void g_strand_post(token_ref obs);
        
        // All methods prefixed by r_queue must be executed
        // by m_exec
        void r_queue_init();              // executed before m_exec->start()
        void r_queue_tick(date_type val); // priority: tick_event
        void r_queue_notify(date_type val, token_id obs, bool fresh);
        void r_queue_synch();             // priority: synch_event
        
        
        node_impl() DELETED;
        
        friend class graph_impl;
        friend class internal_impl;
      }; // TREX::transaction::details::node_impl
      
    } // TREX::transaction::details
  } // TREX::transaction
} // TREX

#endif // H_trex_transaction_node_impl
