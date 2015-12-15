/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Frederic Py.
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
#ifndef H_trex_transaction_private_external_handler
# define H_trex_transaction_private_external_handler

# include "external_impl.hh"
# include <boost/thread/shared_mutex.hpp>

namespace trex {
  namespace transaction {
    namespace details {

      struct ext_cmp {
        bool operator()(SHARED_PTR<external_impl> const &a,
                        SHARED_PTR<external_impl> const &b) const {
          return a->name()<b->name();
        }
      };

      class external_handler {
      public:
        explicit external_handler();
        ~external_handler();
        
        bool r_queue_tick(date_type date);
        bool r_queue_synchronized(date_type date, bool &changed);
        
        bool should_synch() const;
        
        boost::optional<date_type> initial() const;
        boost::optional<date_type> synch_date() const;
        boost::optional<date_type> next_synch() const;
        boost::optional<date_type> synch_target() const;
        boost::optional<date_type> ext_date()   const;
        boost::optional<date_type> graph_date() const;
        
        bool external(utils::symbol const &tl) const;
        
        bool empty() const;
        bool insert(SHARED_PTR<external_impl> const &tl);
        bool update(utils::symbol const &tl, date_type tick,
                    ERROR_CODE &ec);
        
        void clear();
        
      private:
        typedef std::map<SHARED_PTR<external_impl>,
                         boost::optional<date_type>, ext_cmp> tl_map;
        
        mutable boost::shared_mutex m_mtx;
        boost::optional<date_type> m_initial, m_synch, m_ext, m_graph;
        tl_map    m_externals;
      };
      
    } // TREX::transaction::details
  } // TREX::transaction
} // TREX

#endif // H_trex_transaction_private_external_handler
