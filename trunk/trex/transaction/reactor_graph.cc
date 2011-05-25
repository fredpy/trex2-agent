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

graph::graph(Symbol const &name, ext_iterator const &conf, TICK init) 
  :m_name(name), m_currentTick(init) {
  size_t number = add_reactors(conf);
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

void graph::clear() {
  while( !m_reactors.empty() ) {
    syslog()<<"Disconnecting \""<<m_reactors.front()->getName()<<"\" from the graph.";
    m_reactors.front()->isolate();
    m_reactors.pop_front();
  } 
}

long graph::index(graph::reactor_id id) const {
#ifndef DEBUG
  size_t pos = 0;
  // not efficient at all !!!
  details::reactor_set::const_iterator i;
  for(i=m_reactors.begin();
      m_reactors.end()!=i && i->get()!=id; ++i, ++pos);
  return pos;
#else
  return reinterpret_cast<long>(id);
#endif
}

size_t graph::add_reactors(ext_iterator iter) {
  graph *me = this;
  xml_factory::iter_type it = xml_factory::iter_traits::build(iter, me);
  boost::shared_ptr<TeleoReactor> reactor;
  size_t count = 0;
  
  while( m_factory->iter_produce(it, reactor) ) {
    std::pair<details::reactor_set::iterator, bool> 
      ret = m_reactors.insert(reactor);
    if( ret.second ) 
      syslog()<<"Reactor \""<<reactor->getName()<<"\" created.";
    else {
      throw MultipleReactors(*this, **(ret.first));
    }			   
    ++count;
  }
  return count;
}

graph::reactor_id graph::add_reactor(rapidxml::xml_node<> &description) {
  graph *me = this;
  TeleoReactor::xml_arg_type 
    arg = xml_factory::arg_traits::build(description, me);
  boost::shared_ptr<TeleoReactor> tmp(m_factory->produce(arg));
  std::pair<details::reactor_set::iterator, bool> ret = m_reactors.insert(tmp);

  if( ret.second ) 
    syslog()<<"Reactor \""<<tmp->getName()<<"\" created.";
  else {
    throw MultipleReactors(*this, **(ret.first));
  }			   
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
    details::reactor_set::iterator pos = m_reactors.find(r->getName());
    
    if( pos->get()==r ) {
      // clean up relations
      syslog()<<"Destroying reactor \""<<r->getName()<<"\".";
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
      return r;
    }
  }
  return null_reactor();
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


bool graph::assign(graph::reactor_id r, Symbol const &timeline) {
  details::timeline_set::iterator tl = get_timeline(timeline);
  return (*tl)->assign(*r);
}

bool graph::subscribe(reactor_id r, Symbol const &timeline, bool control) {
  details::timeline_set::iterator tl = get_timeline(timeline);
  return (*tl)->subscribe(*r, control);
}
