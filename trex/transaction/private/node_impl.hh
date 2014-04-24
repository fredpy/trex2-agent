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

# include "timeline_impl.hh"

namespace TREX {
  namespace transaction {
    namespace details {

      class node_impl :boost::noncopyable,
      public ENABLE_SHARED_FROM_THIS<node_impl> {
      public:
        ~node_impl();

        
        node_id id() {
          return shared_from_this();
        }
        
        void set_name(utils::symbol const &name) {
          m_name = name;
        }
        utils::symbol const &name() const {
          return m_name;
        }
        
        utils::log::stream syslog(utils::symbol const &ctx,
                                  utils::symbol const &kind) const;
        utils::log_manager &manager() const;
        
        SHARED_PTR<graph_impl> graph() const {
          return m_graph.lock();
        }
        
        bool internal(utils::symbol const &tl) const;
        bool external(utils::symbol const &tl) const;
        
        void provide(utils::symbol const &tl, bool read_only, bool publish_plan);
        void unprovide(utils::symbol const &tl);
        
        void use(utils::symbol const &tl, bool read_only, bool listen_plan);
        void unuse(utils::symbol const &tl);
        
        void notify(TICK date, utils::symbol tl,
                    boost::optional<Observation> o);
        
      private:
        explicit node_impl(WEAK_PTR<graph_impl> const &g);
        
        void isolate(SHARED_PTR<graph_impl> const &g);
        
        typedef std::map<utils::symbol, tl_ref>  internal_set;
        typedef std::map<utils::symbol, ext_ref> external_set;
        
        internal_set m_internals;
        external_set m_externals;
        
        void assigned_sync(tl_ref const &tl);
        void subscribed_sync(ext_ref const &tl);
        
        bool check_internal_sync(utils::symbol tl) const;
        bool check_external_sync(utils::symbol tl) const;
        
        void unprovide_sync(SHARED_PTR<graph_impl> g, utils::symbol tl);
        void unuse_sync(utils::symbol tl);
        
        utils::symbol        m_name;
        WEAK_PTR<graph_impl> m_graph;
        
        friend class graph_impl;

        node_impl() DELETED;
      }; // TREX::transaction::details::node_impl
      
    } // TREX::transaction::details
  } // TREX::transaction
} // TREX

#endif // H_trex_transaction_node_impl
