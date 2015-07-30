/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Frederic Py.
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
#include "ros_reactor.hh"
#include <trex/python/python_thread.hh>
#include <trex/domain/FloatDomain.hh>
#include <trex/domain/IntegerDomain.hh>
#include <trex/domain/BooleanDomain.hh>
#include <trex/domain/StringDomain.hh>

#include <boost/python/def.hpp>
#include <boost/python/stl_iterator.hpp>



using namespace TREX::ROS;
using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::python;
namespace bp=boost::python;
namespace bpt=boost::property_tree;

// structors

ros_reactor::ros_reactor(TeleoReactor::xml_arg_type arg)
:TeleoReactor(arg, false) {
  bpt::ptree::value_type &node(xml_factory::node(arg));
  typedef tl_factory::iter_traits<bpt::ptree::iterator> itt;
  bpt::ptree::iterator ixml = node.second.begin();
  ros_reactor *me = this;
  
  itt::type i = itt::build(ixml, me);
  SHARED_PTR<details::ros_timeline> tl;
  size_t count = 0;
  
  while( m_factory->iter_produce(i, node.second.end(), tl) ) {
    std::pair<tl_set::iterator, bool> ret = m_timelines.insert(tl);
    if( ret.second )
      syslog(null, info)<<"ROS timeline "<<tl->name()<<" created.";
    else
      // should not happen
      throw MultipleInternals(*this, tl->name(), *this);
    ++count;
  }
  
  if( 0==count )
    syslog(log::warn)<<"No connection created: this reactor is useless";
}

ros_reactor::~ros_reactor() {
}


// callbacks

void ros_reactor::handleInit() {
  for(tl_set::const_iterator i=m_timelines.begin(); m_timelines.end()!=i; ++i)
    (*i)->do_init();
}

bool ros_reactor::synchronize() {
  for(tl_set::const_iterator i=m_timelines.begin(); m_timelines.end()!=i; ++i)
    (*i)->do_synchronize();

  return !m_ros->is_shutdown();
}

void ros_reactor::handleRequest(goal_id const &g) {
  tl_set::const_iterator i = m_timelines.find(g->object());

  if( m_timelines.end()!=i ) {
    if( (*i)->controllable() && (*i)->request(g) )
      syslog()<<"Accepted request ["<<g<<"]";
  }
}

void ros_reactor::handleRecall(goal_id const &g) {
  tl_set::const_iterator i = m_timelines.find(g->object());
  
  if( m_timelines.end()!=i )
    (*i)->recall(g);
}


