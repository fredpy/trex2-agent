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
#include "tick_manager.hh"


using namespace TREX::REST;
using namespace TREX::transaction;

namespace bp=boost::property_tree;

namespace {
  
  bp::ptree tick_service(rest_request const &req, tick_manager *mgr) {
    TICK date;
    
    if( req.arg_path().empty() )
      date = mgr->current();
    else {
      std::string arg = req.arg_path().dump();
      try {
        date = boost::lexical_cast<TICK>(arg);
      } catch(...) {
        throw std::runtime_error("Failed to parse "+arg+" as a tick");
      }
    }
    return mgr->json_tick(date);
  }
  
  bp::ptree next_tick(rest_request const &req, tick_manager *mgr) {
    return mgr->json_tick(mgr->current()+1);
  }
  
  
  bp::ptree tick_at(rest_request const &req, tick_manager *mgr) {
    if( req.arg_path().empty() )
      throw std::runtime_error("Missing date argument to "+req.request().path());
    std::cout<<req.arg_path().dump()<<std::endl;
    std::string date_str = my_url_decode(req.arg_path().dump());
    std::cout<<" -> "<<date_str<<std::endl;
    
    
    try {
      graph::date_type date = boost::lexical_cast<graph::date_type>(date_str);
      
      return mgr->json_tick(mgr->tick_at(date));
    } catch(...) {
      throw std::runtime_error("Failed to parse \""+date_str+" as a date.");
    }
  }
  
  class tick_wait :public rest_service {
  public:
    tick_wait(tick_manager &m)
    :rest_service("Wait for next tick."), m_mgr(m) {
      m_conn = m.signal().connect(boost::bind(&tick_wait::new_tick, this, _1));
    }
    
    ~tick_wait() {
      m_conn.disconnect();
      beingDeleted();
    }
    
    void new_tick(TICK cur) {
      haveMoreData();
    }
   
  private:
    
    void handleRequest(rest_request const &req,
                       std::ostream &data,
                       Wt::Http::Response &ans) {
      Wt::Http::ResponseContinuation *cont = req.request().continuation();
      if( cont ) {
        TICK date = boost::any_cast<TICK>(cont->data()),
          cur = m_mgr.current();
        if( date<=cur ) {
          helpers::json_stream json(data);
          ans.setMimeType("application/json");
          
          TREX::utils::write_json(json, m_mgr.json_tick(cur));
        } else {
          cont = ans.createContinuation();
          cont->setData(date);
          cont->waitForMoreData();          
        }
      } else {
        TICK next = m_mgr.current()+1;
        cont = ans.createContinuation();
        cont->setData(next);
        cont->waitForMoreData();
      }
    }
    
    
    
    tick_manager                &m_mgr;
    boost::signals2::connection  m_conn;
  };
  
}

/*
 * class TREX::REST::tick_manager
 */

tick_manager::tick_manager(graph &gr,
                           boost::asio::io_service &io)
:m_agent(gr),m_strand(io) {}

tick_manager::~tick_manager() {}

void tick_manager::populate(service_tree &tree) {
  // TODO add the different services for the tick
  tree.add_handler("tick",
                   new json_direct(boost::bind(&tick_service, _1, this),
                                   "Give tick information.\n"
                                   "If no argument, gives the current tick.\n"
                                   "Example: /rest/tick/1"));
  tree.add_handler("tick/next",
                   new json_direct(boost::bind(&next_tick, _1, this),
                                   "Give next tick information."));
  tree.add_handler("tick/initial",
                   new json_direct(boost::bind(&tick_manager::json_initial, this, _1),
                                   "Give initial tick information."));
  tree.add_handler("tick/final",
                   new json_direct(boost::bind(&tick_manager::json_final, this, _1),
                                   "Give final tick information."));
  tree.add_handler("tick/at",
                   new json_direct(boost::bind(&::tick_at, _1, this),
                                   "Give the largest tick before the given date.\n"
                                   "Example: /tick/at/2013-May-03%2021:17:21"));
  tree.add_handler("tick/rate",
                   new json_direct(boost::bind(&tick_manager::tick_period, this, _1),
                                   "Give the duration between two ticks"));
  
  tree.add_handler("tick/wait", new tick_wait(*this));
}

void tick_manager::new_tick(TICK cur) {
  m_strand.post(boost::bind(&tick_manager::update_sync, this, cur));
}

TICK tick_manager::current() {
  boost::function<TICK ()> fn(boost::bind(&tick_manager::get_sync, this));
  return utils::strand_run(m_strand, fn);
}

TICK tick_manager::tick_at(reactor::date_type const &date) const {
  return m_agent.time_to_tick(date);
}


void tick_manager::update_sync(TICK cur) {
  m_cur = cur;
  if( !m_first )
    m_first = cur;
  m_tick(cur);
}

TICK tick_manager::get_sync() {
  return m_cur;
}

bp::ptree tick_manager::json_tick(TICK val) const {
  bp::ptree ret;
  
  ret.put("value", val);
  ret.put("date", m_agent.date_str(val));
  return ret;
}

bp::ptree tick_manager::json_initial(rest_request const &) const {
  return json_tick(*m_first);
}

bp::ptree tick_manager::json_final(rest_request const &) const {
  return json_tick(m_agent.final_tick());
}

bp::ptree tick_manager::tick_period(rest_request const &) const {
  graph::duration_type rate = m_agent.tick_duration();
  CHRONO::nanoseconds
  ns = CHRONO::duration_cast<CHRONO::nanoseconds>(rate);
  bp::ptree ret;
  
  ret.put("nanoseconds", ns.count());
  ret.put("duration", m_agent.duration_str(1));
  return ret;
}


