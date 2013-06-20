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

# include <map>

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
        
        void set_name(utils::Symbol const &name) {
          m_name = name;
        }
        utils::Symbol const &name() const {
          return m_name;
        }
        
        utils::log::stream syslog(utils::Symbol const &ctx,
                                  utils::Symbol const &kind) const;
        utils::LogManager &manager() const;
        
        SHARED_PTR<graph_impl> graph() const {
          return m_graph.lock();
        }
        
        bool internal(utils::Symbol const &tl) const;
        bool external(utils::Symbol const &tl) const;
        
        
        void provide(utils::Symbol const &tl, bool read_only, bool publish_plan);
        void unprovide(utils::Symbol const &tl);
        void use(utils::Symbol const &tl, bool read_only, bool listen_plan);
        void unuse(utils::Symbol const &tl);
        
      private:
        explicit node_impl(WEAK_PTR<graph_impl> const &g);
        
        void isolate(SHARED_PTR<graph_impl> const &g);
        
        void assigned(tl_ref tl);
        void subscribed(ext_ref tl);
        
        utils::Symbol               m_name;
        WEAK_PTR<graph_impl> m_graph;
        
        typedef std::map<utils::Symbol, tl_ref> internal_map;
        typedef std::map<utils::Symbol, ext_ref> external_map;

        internal_map m_internals;
        external_map m_externals;
        
        void unprovide_sync(SHARED_PTR<graph_impl> g, utils::Symbol tl);
        void unuse_sync(utils::Symbol tl);
        
        bool check_internal_sync(utils::Symbol name) const;
        bool check_external_sync(utils::Symbol name) const;
        
        friend class graph_impl;

        node_impl() DELETED;
      }; // TREX::transaction::details::node_impl
      
    } // TREX::transaction::details
  } // TREX::transaction
} // TREX

#endif // H_trex_transaction_node_impl