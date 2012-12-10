/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2012, MBARI.
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
#ifndef H_trex_ros_ros_reactor
# define H_trex_ros_ros_reactor

# include "ros_subscriber.hh"
# include "ros_client.hh"

# include <trex/transaction/TeleoReactor.hh>

namespace TREX {
  namespace ROS {
    
    class ros_reactor:public TREX::transaction::TeleoReactor {
    public:
      typedef details::ros_timeline::xml_factory ros_factory;


      ros_reactor(TREX::transaction::TeleoReactor::xml_arg_type arg);
      ~ros_reactor();

    protected:
      
    private:
      void handleInit();
      bool synchronize();
      
      void handleRequest(TREX::transaction::goal_id const &g);
      void handleRecall(TREX::transaction::goal_id const &g);

      TREX::utils::SingletonUse<ros_client> m_cli;
      ::ros::NodeHandle                     m_ros;
      TREX::utils::SingletonUse<ros_factory> m_tl_prod;
      
      template<typename Iter>
      size_t add_timelines(Iter from, Iter to);

      typedef std::map<TREX::utils::Symbol, details::ros_timeline::pointer> tl_map;
      tl_map m_tl_conn;

      friend class details::ros_timeline;
    }; // TREX::ros::ros_reactor

    template<typename Iter>
    size_t ros_reactor::add_timelines(Iter from, Iter to) {
      ros_reactor *me = this;
      typename ros_factory::iter_traits<Iter>::type
	it = ros_factory::iter_traits<Iter>::build(from, me);
      details::ros_timeline::pointer tl;
      size_t count = 0;
      bool loop = true;

      while( loop ) {
	try {
	  loop = m_tl_prod->iter_produce(it, to, tl);
	} catch(TREX::utils::Exception const &e) {
	  syslog(TREX::utils::log::null,
		 TREX::utils::log::error)<<"Error while creating ROS timeline: "<<e;
	}
	std::pair<tl_map::iterator, bool>
	  ret = m_tl_conn.insert(std::make_pair(tl->name(), tl));
	if( ret.second ) {
	  syslog(TREX::utils::log::null, 
		 TREX::utils::log::info)<<"ROS timeline \""<<tl->name()<<"\" added";
	  ++count;
	} else 
	  syslog(TREX::utils::log::null, 
		 TREX::utils::log::error)<<"ROS timeline \""<<tl->name()<<"\" already exists.";
      }
      return count;
    }


  } // TREX::ros
} // TREX

#endif // H_trex_ros_ros_reactor
