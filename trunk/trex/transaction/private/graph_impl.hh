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
#ifndef H_trex_transaction_graph_impl
# define H_trex_transaction_graph_impl

# include "../bits/transaction_fwd.hh"
# include "../bits/reactor_policies.hh"
# include "../Tick.hh"

# include <trex/utils/log_manager.hh>
# include <boost/thread/shared_mutex.hpp>
# include <boost/signals2/signal.hpp>


namespace TREX {
  namespace transaction {
    namespace details {
        
      class graph_impl :boost::noncopyable,
      public ENABLE_SHARED_FROM_THIS<graph_impl> {
      public:
        explicit graph_impl(utils::symbol const &name);
        ~graph_impl();
        
        utils::symbol const &name() const {
          return m_name;
        }
        boost::optional<date_type> now() const;

        utils::log_manager &manager() const {
          return *m_log;
        }
        boost::asio::strand &strand() const {
          return *m_strand;
        }
        utils::log::stream syslog(utils::symbol const &who,
                                  utils::symbol const &kind) const;
        
        size_t reactors_size() const;
        SHARED_PTR<node_impl> new_node(utils::symbol const &name,
                                       exec_ref const &queue);
        
        void tick(date_type const &date) {
          strand().dispatch(boost::bind(&graph_impl::g_strand_tick,
                                        shared_from_this(), date));
        }
        tick_sig &on_tick() {
          return m_tick;
        }
        
        bool add_reactor(SHARED_PTR<reactor> r);
        
      private:
        typedef std::set< SHARED_PTR<reactor> >             reactor_set;
        typedef std::set<SHARED_PTR<internal_impl>, tl_cmp> internal_set;
        
        tick_sig            m_tick;
        utils::symbol const m_name;
        utils::singleton::use<utils::log_manager> m_log;
        UNIQ_PTR<boost::asio::strand>             m_strand;
        
        mutable boost::shared_mutex m_date_mtx;
        boost::optional<date_type>  m_date;

        
        mutable boost::shared_mutex m_mtx;
        reactor_set                 m_reactors;
        internal_set                m_internals;
        
        void g_strand_tick(date_type date);
        
        ERROR_CODE g_strand_add(SHARED_PTR<reactor> const &r);
        void g_strand_rm(SHARED_PTR<reactor> r);
        
        SHARED_PTR<internal_impl>
        g_strand_decl(SHARED_PTR<node_impl> const &r,
                     utils::symbol const &tl,
                     transaction_flags gp,
                     ERROR_CODE &ec);
        
        graph_impl() DELETED;
        
        friend class node_impl;
      }; // TREX::transaction::details::graph_impl
      
            
      
    } // TREX::transaction::details
  } // TREX::transaction
} // TREX

#endif // H_trex_transaction_graph_impl
