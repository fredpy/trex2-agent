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

# include <trex/utils/LogManager.hh>

# include <boost/enable_shared_from_this.hpp>
# include <boost/asio/strand.hpp>

namespace TREX {
  namespace transaction {
    namespace details {
      
      class graph_impl :boost::noncopyable,
      public boost::enable_shared_from_this<graph_impl> {
      public:
        typedef utils::log::entry::date_type date_type;
        
        graph_impl();
        explicit graph_impl(utils::Symbol const &name);
        ~graph_impl();
        
        utils::Symbol const &name() const {
          return m_name;
        }
        void name(utils::Symbol const &name) {
          m_name = name;
        }
        void date(date_type const &d);
        boost::optional<date_type> date() const;
        
        utils::log::stream syslog(utils::Symbol const &ctx,
                                  utils::Symbol const &kind) const;
        
        utils::LogManager &manager() const {
          return *m_log;
        }
        boost::asio::strand &strand() const {
          return *m_strand;
        }
        
      private:
        void set_date_sync(date_type date);
        
        mutable utils::SharedVar< boost::optional<date_type> > m_date;
        utils::Symbol                          m_name;
        utils::SingletonUse<utils::LogManager> m_log;
        UNIQ_PTR<boost::asio::strand>          m_strand;
      }; // TREX::transaction::details::graph_impl
      
    } // TREX::transaction::details
  } // TREX::transaction
} // TREX

#endif // H_trex_transaction_graph_impl