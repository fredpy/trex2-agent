#include "EuropaReactor.hh"

#include "bits/europa_convert.hh"

#include "PLASMA/PlanDatabase.hh"
#include "PLASMA/Token.hh"
#include "PLASMA/TokenVariable.hh"

using namespace TREX::europa;
using namespace TREX::transaction;
using namespace TREX::utils;

/*
 * class TREX::europa::EuropaReactor
 */ 

// structors 

EuropaReactor::EuropaReactor(xml_arg_type arg)
  :TeleoReactor(arg, false), m_assembly(*this) {
  bool found;
  std::string short_nddl = getName().str()+".nddl",
    long_nddl = getGraphName().str()+short_nddl, 
    nddl;
  // locate nddl model 
  //   - 1st look for <agent>.<reactor>.nddl
  nddl = manager().use(long_nddl, found);
  if( !found ) {
    // if not found look for jsut <reactor>.nddl
    nddl = manager().use(short_nddl, found);
    if( !found )
      // no model found for this reactor 
      throw ReactorException(*this, "Unable to locate \""+long_nddl+"\" or \""+
			     short_nddl+"\"");
  }
  // Load the model
  if( !m_assembly.playTransaction(nddl) )
    throw ReactorException(*this, "model in "+nddl+" is inconsistent.");
  
  if( !m_assembly.plan_db()->isClosed() ) {
    syslog("WARN")<<"Plan Database is not closed:\n"
		  <<"\tClosing it now!!\n";
    m_assembly.plan_db()->close();
  }

  // Now load solver configuration 
  std::string solver_cfg = parse_attr<std::string>(xml_factory::node(arg), 
						   "solverConfig");  
  if( solver_cfg.empty() )
    throw XmlError(xml_factory::node(arg), "solverConfig attribute is empty");
  // MISSING start : how to implement this ?

  // MISSIN end
  // finally I identify the external end internal timelines
  std::list<EUROPA::ObjectId> objs;
  m_assembly.trex_timelines(objs);
  
  for(std::list<EUROPA::ObjectId>::const_iterator it=objs.begin(); 
      objs.end()!=it; ++it) {
    EUROPA::LabelStr name = (*it)->getName();
    EUROPA::LabelStr mode = m_assembly.mode(*it);
    Symbol trex_name(name.c_str());
    
    if( Assembly::EXTERNAL_MODE==mode || Assembly::OBSERVE_MODE==mode ) 
      use(trex_name, Assembly::OBSERVE_MODE!=mode);      
    else if( Assembly::INTERNAL_MODE==mode ) {
      provide(trex_name);
      if( !isInternal(trex_name) ) {
	// Formally it would be better to demote it as External
	// but for now we will just hide this timeline to the rest of the world
	syslog("WARNING")<<"Unable to declare "<<trex_name<<" as Internal ...\n"
			 <<"\t making it Private.";
      } 
    } else if( Assembly::IGNORE_MODE==mode ) {
      m_assembly.ignore(*it);
    } 
    // everything else is just private ...
  }
}

EuropaReactor::~EuropaReactor() {}

// callbacks

//  - TREX transaction callback 

void EuropaReactor::notify(Observation const &o) {
  EUROPA::TokenId tok = m_assembly.convert(o, false);
  // restrict start to current tick
  tok->start()->restrictBaseDomain(EUROPA::IntervalIntDomain(getCurrentTick(),
							     getCurrentTick()));
  tok->end()->restrictBaseDomain(EUROPA::IntervalIntDomain(getCurrentTick()+1,
							   PLUS_INFINITY));
  // m_core->notify(tok);
}

void EuropaReactor::handleRequest(goal_id const &g) {
  EUROPA::TokenId tok = m_assembly.convert(*g, false);
  // restrict start, duration and end
  try {
    details::europa_restrict(tok->start(), g->getStart());
    details::europa_restrict(tok->duration(), g->getDuration());
    details::europa_restrict(tok->end(), g->getEnd());
  } catch(DomainExcept const &de) {
    syslog("ERROR")<<"Failed to restrict goal "<<g->object()<<'.'<<g->predicate()
		   <<'['<<g<<"] temporal attributes: "<<de;
    return;
  }
  m_internal_goals[tok->getKey()] = g;
  if( m_assembly.inactive() )
    m_assembly.mark_active();
}

void EuropaReactor::handleRecall(goal_id const &g) {
  for(europa_mapping::iterator i=m_internal_goals.begin(); 
      m_internal_goals.end()!=i; ++i)
    if( i->second==g ) {
      EUROPA::eint key = i->first;
      m_internal_goals.erase(i);

      // Need to notify the europa solver
      // m_core->recall(key);
      return;
    }
}

//  - TREX execution callbacks

void EuropaReactor::handleInit() {
}

void EuropaReactor::handleTickStart() {
  if( getCurrentTick()==getInitialTick() )
    reset_deliberation();
}

bool EuropaReactor::synchronize() {
}

bool EuropaReactor::hasWork() {
  if( m_assembly.active() )
    return true;
  /* extra tests in between */
    
  return true;
}

void EuropaReactor::resume() {
}

//  - Europa interface callbacks

void EuropaReactor::removed(EUROPA::TokenId const &tok) {
  europa_mapping::iterator i = m_external_goals.find(tok->getKey());
  if( m_external_goals.end()!=i ) 
    m_external_goals.erase(i);
  else {
    i = m_internal_goals.find(tok->getKey());
    m_internal_goals.erase(i);
  }
}

void EuropaReactor::request(Symbol const &tl, 
				EUROPA::TokenId const &tok) {
  europa_mapping::iterator i = m_external_goals.find(tok->getKey());
  if( m_external_goals.end()==i ) {
    Goal myGoal(tl, tok->getUnqualifiedPredicateName().toString());
    
    // build the goal

    goal_id request = postGoal(myGoal);
    if( request )
      m_external_goals[tok->getKey()] = request;
  }  
}

void EuropaReactor::recall(EUROPA::TokenId const &tok) {
  europa_mapping::iterator i = m_external_goals.find(tok->getKey());
  if( m_external_goals.end()!=i ) {
    goal_id g = i->second;
    m_external_goals.erase(i);
    postRecall(g);
  }
}

void EuropaReactor::notify(Symbol const &tl, 
			   EUROPA::TokenId const &tok) {
  Observation obs(tl, tok->getUnqualifiedPredicateName().toString());
  
  // populate the observation 

  postObservation(obs);
}

