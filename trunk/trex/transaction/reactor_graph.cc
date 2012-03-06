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
#include "reactor_graph.hh"
#include "TeleoReactor.hh"

using namespace TREX::transaction;
using namespace TREX::utils;

/*
 * class TREX::transaction::GraphException
 */
GraphException::GraphException(graph const &g, std::string const &msg) throw()
  :TREX::utils::Exception(g.getName().str()+": "+msg) {}

GraphException::GraphException(graph const &g, std::string const &who,
			       std::string const &msg) throw()
  :TREX::utils::Exception(g.getName().str()+"."+who+": "+msg) {}


/*
 * class TREX::transaction::MultipleReactors
 */
MultipleReactors::MultipleReactors(graph const &g, TeleoReactor const &r) throw()
  :GraphException(g, "Multiple reactors with the same name \""+
		  r.getName().str()+"\"") {}

/*
 * class TREX::transaction::graph
 */

// structors :

graph::graph(Symbol const &name, TICK init) 
  :m_name(name), m_currentTick(init) {
}

graph::graph(Symbol const &name, boost::property_tree::ptree &conf, TICK init) 
  :m_name(name), m_currentTick(init) {
  size_t number = add_reactors(conf);
  syslog()<<"Created "<<number<<" reactors.";
}

graph::~graph() {
  clear();
}

TREX::utils::internals::LogEntry graph::syslog(std::string const &context) const {
  std::ostringstream oss;
  oss<<m_currentTick<<"]["<<m_name.str();
  if( !context.empty() )
    oss<<"."<<context;

  return m_log->syslog(oss.str());
}

bool graph::is_isolated(graph::reactor_id r) const {
  return m_quarantined.find(r->getName())!=m_quarantined.end();
}


void graph::clear() {
  while( !m_reactors.empty() ) {
    syslog()<<"Disconnecting \""<<m_reactors.front()->getName()<<"\" from the graph.";
    m_reactors.front()->isolate();
    m_reactors.pop_front();
  } 
  m_quarantined.clear();
}

long graph::index(graph::reactor_id id) const {
  size_t pos = 0;
  // not efficient at all !!!
  details::reactor_set::const_iterator i;
  for(i=m_reactors.begin();
      m_reactors.end()!=i && i->get()!=id; ++i, ++pos);
  return pos;
}

graph::reactor_id graph::add_reactor(boost::property_tree::ptree::value_type &description) {
  graph *me = this;
  TeleoReactor::xml_arg_type 
    arg = xml_factory::arg_traits::build(description, me);
  boost::shared_ptr<TeleoReactor> tmp(m_factory->produce(arg));
  std::pair<details::reactor_set::iterator, bool> ret = m_reactors.insert(tmp);

  if( ret.second ) 
    syslog()<<"Reactor \""<<tmp->getName()<<"\" created.";
  else
    throw MultipleReactors(*this, **(ret.first));			   
  return ret.first->get();
}

graph::reactor_id graph::add_reactor(graph::reactor_id r) {
  boost::shared_ptr<TeleoReactor> tmp(r);  
  std::pair<details::reactor_set::iterator, bool> ret = m_reactors.insert(tmp);
  // As it is an internal call make is silent for now ...
  return ret.first->get();
}

bool graph::kill_reactor(graph::reactor_id r) {
  if( null_reactor()!=r ) {
    details::reactor_set::iterator pos = m_reactors.find(r->getName()),
      pos_q = m_quarantined.find(r->getName());
    
    if( pos->get()==r ) {
      // clean up relations
      syslog()<<"Destroying reactor \""<<r->getName()<<"\".";
      if( pos_q->get()==r )
	m_quarantined.erase(pos_q);
      else 
        r->isolate();
      m_reactors.erase(pos);
      return true;
    }
  }
  return false;
}

graph::reactor_id graph::isolate(graph::reactor_id r) const {
  if( null_reactor()!=r ) {
    details::reactor_set::const_iterator pos = m_reactors.find(r->getName());
    if( pos->get()==r ) {
      syslog()<<"Putting reactor\""<<r->getName()<<"\" in quarantine.";
      r->isolate();
      m_quarantined.insert(*pos);
      return r;
    }
  }
  return null_reactor();
}

size_t graph::cleanup() {
  size_t ret =0;
  for(; !m_quarantined.empty(); ++ret)
    kill_reactor(m_quarantined.front().get());
  return ret;
}  

details::timeline_set::iterator graph::get_timeline(Symbol const &tl) {
  details::timeline *cand = new details::timeline(m_currentTick, tl);
  std::pair<details::timeline_set::iterator, bool>
    ret = m_timelines.insert(cand);
  if( ret.second ) 
    syslog()<<"Timeline \""<<tl.str()<<"\" created.";
  else
    delete cand;
  return ret.first;
}

graph::size_type graph::count_relations() const {
  size_t res = 0;
  for(details::timeline_set::const_iterator i=m_timelines.begin();
      m_timelines.end()!=i; ++i )
    res += (*i)->size();
  return res;
}


bool graph::assign(graph::reactor_id r, Symbol const &timeline, bool controllable) {
  details::timeline_set::iterator tl = get_timeline(timeline);
  try {
    internal_check(r, **tl);
    return (*tl)->assign(*r, controllable);
  } catch(timeline_failure const &err) {
    return r->failed_internal(timeline, err);
  }
}

bool graph::subscribe(reactor_id r, Symbol const &timeline, bool control) {
  details::timeline_set::iterator tl = get_timeline(timeline);
  try {
    external_check(r, **tl);
    return (*tl)->subscribe(*r, control);
  } catch(timeline_failure const &err) {
    return r->failed_external(timeline, err);
  }
}

/*
 * class TREX::transaction::graph::timelines_listener
 */

// structors 

graph::timelines_listener::timelines_listener(graph &g)
  :m_graph(g) {
    m_graph.m_listeners.insert(this);
}

graph::timelines_listener::timelines_listener(graph::xml_factory::argument_type const &arg)
  :m_graph(*(arg.second)){
  m_graph.m_listeners.insert(this);  
}

graph::timelines_listener::~timelines_listener() {
  m_graph.m_listeners.erase(this);
}
  
// manpipulators
void graph::timelines_listener::initialize() {
  for(details::timeline_set::const_iterator tl=m_graph.m_timelines.begin();
      m_graph.m_timelines.end()!=tl; ++tl)
    declared(**tl);
}

