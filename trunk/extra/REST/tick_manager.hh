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
#ifndef H_trex_rest_tick_manager 
# define H_trex_rest_tick_manager 

# include <trex/transaction/reactor.hh>
# include "REST_service.hh"

# include <boost/signals2/signal.hpp>

namespace TREX {
  namespace REST {
    
    class tick_manager {
    public:
      typedef boost::signals2::signal<void (transaction::TICK)> tick_event;
      
      tick_manager(transaction::graph &gr, boost::asio::io_service &io);
      ~tick_manager();
      
      void populate(service_tree &tree);
      
      void new_tick(transaction::TICK cur);
      transaction::TICK current();
      
      transaction::TICK tick_at(transaction::reactor::date_type const &date) const;
      
      boost::property_tree::ptree json_tick(transaction::TICK val) const;
      boost::property_tree::ptree json_initial(rest_request const &) const;
      boost::property_tree::ptree json_final(rest_request const &) const;
      boost::property_tree::ptree tick_period(rest_request const &) const;
      
      tick_event &signal() {
        return m_tick;
      }
      
    private:
      transaction::graph  &m_agent;
      boost::asio::strand  m_strand;
      transaction::TICK    m_cur;
      boost::optional<transaction::TICK> m_first;
      
      tick_event           m_tick;
      
      void update_sync(transaction::TICK cur);
      transaction::TICK get_sync();
    };
  
    
  }
}

#endif
