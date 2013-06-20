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
#ifndef H_trex_transaction_timeline_impl
# define H_trex_transaction_timeline_impl

# include "../bits/transaction_fwd.hh"

namespace TREX {
  namespace transaction {
    namespace details {
      
      class external_impl;
      
      typedef SHARED_PTR<external_impl> ext_ref;
      
      class internal_impl :boost::noncopyable, public boost::enable_shared_from_this<internal_impl> {
      public:
        internal_impl(utils::Symbol const &name, WEAK_PTR<graph_impl> const &g);
        ~internal_impl();
        
        utils::Symbol const &name() const {
          return m_name;
        }
        
        node_id owner() const;
        
        bool accept_goals() const;
        bool publish_plan() const;
        
        SHARED_PTR<graph_impl> graph() const {
          return m_graph.lock();
        }
        
      private:
        utils::Symbol               m_name;
        WEAK_PTR<graph_impl> m_graph;
        
        transaction_flags           m_flags;
        node_id                     m_owner;
      
        node_id owner_sync() const;
        bool reset_sync();
        /**
         *
         * @retval true if either the owner or the flags have been modified
         * @retval false otherwise
         */
        bool set_sync(SHARED_PTR<node_impl> const &n, transaction_flags const &fl);
        
        
        
        friend class graph_impl;
        internal_impl() DELETED;
      }; // TREX::transaction::details::internal_impl
      
      
      class external_impl :boost::noncopyable, public boost::enable_shared_from_this<external_impl> {
      public:
        external_impl(SHARED_PTR<node_impl> cli, tl_ref tl, transaction_flags const &fl);
        ~external_impl();
        
        utils::Symbol const &name() const {
          return m_timeline->name();
        }
        
        bool accept_goals() const;
        bool publish_plan() const;
        
        SHARED_PTR<graph_impl> graph() const;

        
      private:
        tl_ref            m_timeline;
        node_id           m_client;
        transaction_flags m_flags;
        
        external_impl() DELETED;
      }; // TREX::transaction::details::external_impl
      

    } // TREX::transaction::details
  } // TREX::transaction
} // TREX

#endif // H_trex_transaction_timeline_impl
