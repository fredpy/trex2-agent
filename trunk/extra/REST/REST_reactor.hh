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


namespace TREX {
  namespace REST {
    
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
      boost::property_tree::ptree tick_period(req_info const &req) const;
      boost::property_tree::ptree get_tick(req_info const &req) const;
      boost::property_tree::ptree tick_at(req_info const &req) const;
      boost::property_tree::ptree next_tick(req_info const &) const {
        return tick_info(getCurrentTick()+1);
      }
      boost::property_tree::ptree initial_tick(req_info const &) const {
        return tick_info(getInitialTick());
      }
      boost::property_tree::ptree final_tick(req_info const &) const {
        return tick_info(getFinalTick());
      }
            
      boost::property_tree::ptree timelines(req_info const &req);
      boost::property_tree::ptree timeline(req_info const &req);
      boost::property_tree::ptree manage_goal(req_info const &req);

      boost::property_tree::ptree get_timeline(std::string name);
      
      void add_tl(utils::Symbol const &tl);
      void remove_tl(utils::Symbol const &tl);
      
      template<typename Ret>
      Ret strand_run(boost::function<Ret ()> const &f) {
        boost::packaged_task<Ret> tsk(f);
        boost::unique_future<Ret> result = tsk.get_future();
        
        m_strand->post(boost::bind(&boost::packaged_task<Ret>::operator(),
                                  boost::ref(tsk)));
        return result.get();
      }
      
      REST_service m_services;
  
      
      UNIQ_PTR<Wt::WServer>           m_server;

      UNIQ_PTR<boost::asio::strand>   m_strand;
      
      typedef std::set<utils::Symbol> tl_set;
      tl_set m_timelines;
    };
    
  }
}

#endif // H_trex_REST_reactor