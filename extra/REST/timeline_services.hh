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
#ifndef H_trex_rest_timeline_services
# define H_trex_rest_timeline_services

# include "REST_service.hh"

# include <trex/utils/SharedVar.hh>

namespace TREX {
  namespace REST {
    
    class TimelineHistory;
    
    class timeline_list_service :public rest_service {
    public:
      timeline_list_service(boost::weak_ptr<TimelineHistory> const &ref)
      :rest_service("List all the timelines"), m_entry(ref) {}
      ~timeline_list_service() {
        beingDeleted();
      }
      
    private:
      void handleRequest(rest_request const &req,
                         std::ostream &data,
                         Wt::Http::Response &ans);
      
      boost::weak_ptr<TimelineHistory> m_entry;
    };
    
    class timeline_service :public rest_service {
    public:
      timeline_service(boost::weak_ptr<TimelineHistory> const &ref)
      :rest_service("Give state history of the timeline.\nOptioanl args are:\n"
                    " - format: format indicator (tick or date)\n"
                    " - from: initial tick of the requested range\n"
                    " - to: last tick of the requested range"), m_entry(ref) {}
      ~timeline_service() {
        beingDeleted();
      }
      
    private:
      
      void handleRequest(rest_request const &req,
                         std::ostream &data,
                         Wt::Http::Response &ans);
      
      boost::weak_ptr<TimelineHistory> m_entry;
    };
    
    class goals_service :public rest_service {
    public:
      goals_service(boost::weak_ptr<TimelineHistory> const &ref)
      :rest_service("List all the goals received"), m_entry(ref) {}
      ~goals_service() {
        beingDeleted();
      }
      
    private:
      void handleRequest(rest_request const &req,
                         std::ostream &data,
                         Wt::Http::Response &ans);
      
      boost::weak_ptr<TimelineHistory> m_entry;
      
    };
    
    class goal_service :public rest_service {
    public:
      goal_service(boost::weak_ptr<TimelineHistory> const &ref, std::string prefix)
      :rest_service("POST: post the attached goal to trex.\n"
                    "DELETE: request the cancelation of the given goal.\n"
                    "GET: get a description of an existing goal."), m_entry(ref) {}
      ~goal_service() {
        beingDeleted();
      }
      
    private:
      void handleRequest(rest_request const &req,
                         std::ostream &data,
                         Wt::Http::Response &ans);
      
      std::string file_name();
      
      boost::weak_ptr<TimelineHistory> m_entry;
      std::string const m_prefix;
      TREX::utils::SharedVar<size_t> m_counter;
    };
  }
}

#endif