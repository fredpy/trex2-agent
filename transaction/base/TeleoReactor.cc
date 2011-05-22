/** @file TeleoReactor.cc
 * @brief Provides implementation for TeleoReactor
 * 
 * @author Conor McGann @& Frederic Py <fpy@mbari.org>
 * @ingroup transaction
 */
#include <utility>
#include <cmath>

#include "TeleoReactor.hh"

using namespace TREX::transaction;
using namespace TREX::utils;

/*
 * class TREX::transaction::ReactorException
 */
ReactorException::ReactorException(TeleoReactor const &r, 
				   std::string const &msg) throw()
  :GraphException(r.m_graph, r.getName().str(), msg) {}


/*
 * class TREX::transaction::DispatchError
 */
// statics 
std::string DispatchError::buil_msg(goal_id const &g, std::string const &msg) throw() {
  std::ostringstream oss;
  oss<<"While dispatching ";
  if( !!g ) 
    oss<<g->object()<<'.'<<g->predicate();
  oss<<'['<<g<<"]: "<<msg;
  return oss.str();
}



/* 
 * class TREX::transaction::details::external
 */

bool details::external::cmp_goals(IntegerDomain const &a, IntegerDomain const &b) {
  // sorting order
  //   - based on upperBound
  //   - if same upperBound : sorted base on lower bound
  //
  // this way I can safely update lower bounds without impacting tokens order
  return a.upperBound()<b.upperBound() ||
    ( a.upperBound()==b.upperBound() && a.lowerBound()<b.lowerBound() );
}


// structors

details::external::external() {
  m_pos = m_last; // may not be necessary but I just want  ot be sure ...
}

details::external::external(details::external const &other) 
  :m_pos(other.m_pos), m_last(other.m_last) {}

details::external::external(details::external_set::iterator const &pos,
			    details::external_set::iterator const &last)
  :m_pos(pos), m_last(last) {
  next_active();
}

// manipulators

details::goal_queue::iterator details::external::lower_bound(IntegerDomain const &dom) {
  details::goal_queue::iterator i = m_pos->second.begin();
  
  for(; m_pos->second.end()!=i && cmp_goals((*i)->getStart(), dom); ++i);
  return i;
}

// modifiers 

bool details::external::post_goal(goal_id const &g) {
  IntegerDomain const &g_start(g->getStart());
  // locate goal position
  details::goal_queue::iterator i = lower_bound(g_start);

  // check that it is not already posted
  for( ; m_pos->second.end()!=i && !cmp_goals(g_start, (*i)->getStart()); ++i)
    if( g==(*i) )
      return false;
  // insert the new goal
  m_pos->second.insert(i, g);
  return true;
}

void details::external::dispatch(TICK current, details::goal_queue &sent) {
  details::goal_queue::iterator i=m_pos->second.begin();
  IntegerDomain dispatch_w = m_pos->first.dispatch_window(current);
  
  for( ; m_pos->second.end()!=i && (*i)->startsBefore(dispatch_w.upperBound());  ) {
    if( (*i)->startsAfter(current) ) {
      // Need to check for dispatching
      if( m_pos->first.accept_goals() ) {
	syslog()<<"Dispatching "<<(*i)->predicate()<<'['<<(*i)<<"] on \""
	 	<<m_pos->first.name()<<"\".";
	m_pos->first.request(*i);
	i = m_pos->second.erase(i);
      } else 
	++i;
    } else {
      syslog()<<"Goal "<<(*i)->predicate()<<'['<<(*i)<<"] is in the past !";
      i = m_pos->second.erase(i);
    }
  } 
}

void details::external::recall(goal_id const &g) {
  IntegerDomain const &g_start(g->getStart());
  // locate goal position
  details::goal_queue::iterator i = lower_bound(g_start);
  for( ; m_pos->second.end()!=i && !cmp_goals(g_start, (*i)->getStart()); ++i)
    if( g==(*i) ) {
      // was still pending => just remove it
      m_pos->second.erase(i);
      return;
    }
  // not found => send a recall
  m_pos->first.recall(g);
}
 
details::external &details::external::operator++() {
  if( valid() ) {
    ++m_pos;
    next_active();
  }
  return *this;
}

void details::external::next_active() {
  while( valid() && !m_pos->first.active() )
    ++m_pos;
}

// observers 

bool details::external::operator==(details::external const &other) const {
  if ( valid() )
    return m_pos == other.m_pos;
  else 
    return !other.valid();
}

TREX::utils::internals::LogEntry details::external::syslog() {
  m_pos->first.client().syslog(m_pos->first.name().str());
}

/*
 * class TREX::transaction::TeleoReactor
 */

// structors


TeleoReactor::TeleoReactor(TeleoReactor::xml_arg_type &arg, bool loadTL)
  :m_inited(false), m_graph(*(arg.second)), 
   m_name(parse_attr<Symbol>(xml_factory::node(arg), "name")),
   m_latency(parse_attr<TICK>(xml_factory::node(arg), "latency")),
   m_maxDelay(0),
   m_lookahead(parse_attr<TICK>(xml_factory::node(arg), "lookahead")),
   m_nSteps(0) {
  if( loadTL ) {
    rapidxml::xml_node<> &node(xml_factory::node(arg));
    Symbol tl_name;
    
    for(ext_iterator iter(node, "config"); iter.valid(); ++iter) {
      if( is_tag(*iter, "External") ) {
	tl_name = parse_attr<Symbol>(*iter, "name");
	if( tl_name.empty() )
	  throw XmlError(*iter, "Timelines cannot have an empty name");
	use(tl_name, parse_attr<bool>(true, *iter, "goals"));
      } else if( is_tag(*iter, "Internal") ) {
	tl_name = parse_attr<Symbol>(*iter, "name");
	if( tl_name.empty() )
	  throw XmlError(*iter, "Timelines cannot have an empty name");
	provide(tl_name);
      }
    }
  }
}
   
TeleoReactor::TeleoReactor(graph *owner, Symbol const &name, 
			   TICK latency, TICK lookahead, bool log)
  :m_inited(false), m_graph(*owner), m_name(name),
   m_latency(latency), m_maxDelay(0), m_lookahead(lookahead),
   m_nSteps(0) {}

TeleoReactor::~TeleoReactor() {
  isolate();
}

// observers 

bool TeleoReactor::isInternal(TREX::utils::Symbol const &timeline) const {
  return m_internals.end()!=m_internals.find(timeline);
}

bool TeleoReactor::isExternal(TREX::utils::Symbol const &timeline) const {
  return m_externals.end()!=m_externals.find(timeline);
}

details::external TeleoReactor::ext_begin() {
  return details::external(m_externals.begin(), m_externals.end());
}

details::external TeleoReactor::ext_end() {
  return details::external(m_externals.end(), m_externals.end());
}
  

// modifers/callbacks

double TeleoReactor::workRatio() {
  if( hasWork() ) {
    double ret = m_deadline;
    ret -= getCurrentTick();
    if( ret<=0.0 ) 
      ret = m_nSteps+1;
    else {
      ret += 1.0;
      ret *= m_nSteps+1;
    }
    return 1.0/ret;
  } else {
    m_nSteps = 0;
    m_deadline = getCurrentTick()+1+getLatency();
    return NAN;
  }
}

void TeleoReactor::postObservation(Observation const &obs) {
  internal_set::iterator i = m_internals.find(obs.object());
  
  if( m_internals.end()==i )
    throw SynchronizationError(*this, "attempted to post observation on "+
			       obs.object().str()+" which is not Internal.");
  
  (*i)->postObservation(getCurrentTick(), obs);
  syslog("ASSERT")<<obs;
}

bool TeleoReactor::postGoal(goal_id const &g) {
  if( !g ) 
    throw DispatchError(*this, g, "Invalid goal Id");
    
  details::external tl(m_externals.find(g->object()), m_externals.end());
  
  if( !tl.valid() )
    return tl.post_goal(g);
  else 
    throw DispatchError(*this, g, "Goals can only be posted on External timelines");
}

goal_id TeleoReactor::postGoal(Goal const &g) {
  goal_id tmp(new Goal(g));
  
  if( postGoal(tmp) )
    return tmp;
  else {
    // should never happen !?
    return goal_id();
  }
}

bool TeleoReactor::postRecall(goal_id const &g) {
  details::external tl(m_externals.find(g->object()), m_externals.end());

  if( tl.valid() ) {
    tl.recall(g);
    return true;
  }
  return false;
}

void TeleoReactor::initialize(TICK final) {
  if( m_inited )
    throw ReactorException(*this, "Attempted to initialize this reactor twice.");
  m_initialTick = getCurrentTick();
  m_finalTick   = final;
  syslog()<<" execution latency is "<<getExecLatency();
  handleInit();   // allow derived class intialization
  m_inited = true;
}

void TeleoReactor::newTick() {
  handleTickStart(); // allow derived class processing

  // Dispatched goals management
  details::external i = ext_begin();
  details::goal_queue dispatched; // store the goals that got dispatched on this tick ...
				  // I do nothing with it for now  
  // Manage goal dispatching
  for( ; i.valid(); ++i )
    i.dispatch(getCurrentTick(), dispatched);
}

void TeleoReactor::doNotify() {
  for(external_set::iterator i = m_externals.begin();
      m_externals.end()!=i; ++i) 
    if( i->first.lastObsDate()==getCurrentTick() )
      notify( i->first.lastObservation() );
}
  

bool TeleoReactor::doSynchronize() {
  try {
    // Update the timelines here !
    if( synchronize() )
      return true;
  } catch(Exception const &e) {
    syslog("SYNCH")<<"Exception caught: "<<e;
  } catch(std::exception const &se) {
    syslog("SYNCH")<<"C++ exception caught: "<<se.what();
  } catch(...) {
    syslog("SYNCH")<<"Unknown exception caught.";
  }
  syslog("ERROR")<<"Failed to synchronize.";
  return false;
}

void TeleoReactor::step() {
  resume();
  m_nSteps += 1;
}

void TeleoReactor::use(TREX::utils::Symbol const &timeline, bool control) {
  if( !m_graph.subscribe(this, timeline, control) )
    if( isInternal(timeline) )
      syslog("WARN")<<"External declaration of the Internal timeline \""
	      <<timeline.str()<<"\"";
    else
      syslog("WARN")<<"Multiple External declarations of timeline \""
	      <<timeline.str()<<"\"";
}

void TeleoReactor::provide(TREX::utils::Symbol const &timeline) {
  if( !m_graph.assign(this, timeline) )
    syslog("WARN")<<"Promoted \""<<timeline.str()<<"\" from External to Internal.";
}
  
void TeleoReactor::clear_internals() {
  while( !m_internals.empty() ) {
    m_internals.front()->unassign(getCurrentTick());
  }
}

void TeleoReactor::clear_externals() {
  while( !m_externals.empty() ) {
    Relation r = m_externals.begin()->first;
    r.unsubscribe();
  }
}

void TeleoReactor::assigned(details::timeline *tl) {
  m_internals.insert(tl);
  syslog()<<"Declared \""<<tl->name()<<"\".";
}

void TeleoReactor::unassigned(details::timeline *tl) {
  internal_set::iterator i = m_internals.find(tl);
  m_internals.erase(i);
  syslog()<<"Undeclared \""<<tl->name()<<"\".";
}

void TeleoReactor::subscribed(Relation const &r) {
  external_set::value_type tmp;
  tmp.first = r;
  m_externals.insert(tmp);
  latency_updated(0, r.latency());
  syslog()<<"Subscribed to \""<<r.name()<<"\".";
}

void TeleoReactor::unsubscribed(Relation const &r) {
  external_set::iterator i = m_externals.find(Relation::get_id(r));
  // No need to control that i is valid 
  //    - this call comes from timeline::unsubscribe so
  //      which calls it only to the client of r -> me
  if( !i->second.empty() ) {
    latency_updated(r.latency(), 0);
  }
  // remove this relation
  m_externals.erase(i);
  syslog()<<"Unsubscribed from \""<<r.name()<<"\".";
}


void TeleoReactor::latency_updated(TICK old_l, TICK new_l) {
  TICK prev = m_maxDelay;

  if( new_l>m_maxDelay )     
    m_maxDelay = new_l;    
  else if( old_l==m_maxDelay ) {
    // special case : the updated value is smaller and the old value is 
    //                equal to my former exec delay
    //                this means that on of the timelines that were
    //                constraining my maxDelay has just reduced its latency
    m_maxDelay = new_l;
    // I need to check what is the new maximum from there
    for(details::external i = ext_begin(); i.valid(); ++i)
      m_maxDelay = std::max(m_maxDelay, i->latency());
  }
  if( m_maxDelay!=prev ) {
    // It may be anoying on the long run but for now I will log when this
    // exec latency changes
    syslog()<<" Execution latency updated from "<<prev<<" to "<<m_maxDelay;
    // Notify all the reactors that depend on me
    for(internal_set::iterator i=m_internals.begin(); m_internals.end()!=i; ++i) 
      (*i)->latency_update(getLatency()+prev);
  }
}
  
