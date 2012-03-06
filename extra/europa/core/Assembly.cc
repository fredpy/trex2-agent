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
#include "Assembly.hh"
#include "private/Schema.hh"
#include "private/CurrentState.hh"

#include "ReactorPropagator.hh"
#include "DeliberationFilter.hh"
#include "SynchronizationManager.hh"

#include <PLASMA/ModuleConstraintEngine.hh>
#include <PLASMA/ModulePlanDatabase.hh>
#include <PLASMA/ModuleRulesEngine.hh>
#include <PLASMA/ModuleTemporalNetwork.hh>
#include <PLASMA/ModuleSolvers.hh>
#include <PLASMA/ModuleNddl.hh>

#include <PLASMA/RulesEngine.hh>
#include <PLASMA/Propagators.hh>
#include <PLASMA/Schema.hh>
#include <PLASMA/NddlInterpreter.hh>

#include <PLASMA/Timeline.hh>
#include <PLASMA/TokenVariable.hh>

#include <PLASMA/XMLUtils.hh>
#include <PLASMA/Debug.hh>


using namespace TREX::europa;

namespace {
  
  struct is_not_merged {
    is_not_merged(bool filt):m_filt(filt) {}
    
    bool operator()(EUROPA::TokenId const &tok) const {
      return m_filt || !tok->isMerged();
    }
    
    bool is_fact(EUROPA::TokenId const &tok) const {
      bool ret = tok->isFact();
      
      if( !( ret || m_filt ) ) {
        if( tok->isActive() ) {
          EUROPA::TokenSet const &merged = tok->getMergedTokens();
          for(EUROPA::TokenSet::const_iterator i=merged.begin();
              merged.end()!=i; ++i)
            if( (*i)->isFact() )
              return true;
        }
      }
      return ret; 
    }
    
    bool is_goal(EUROPA::TokenId const &tok) const {
      bool ret = tok->getState()->baseDomain().isMember(EUROPA::Token::REJECTED);
        
      if( !(ret || m_filt) ) {
        if( tok->isActive() ) {
          EUROPA::TokenSet const &merged = tok->getMergedTokens();
          for(EUROPA::TokenSet::const_iterator i=merged.begin();
              merged.end()!=i; ++i)
            if( is_goal(*i) )
              return true;
        }
      }
      return ret;
    }
    
    void merged(EUROPA::TokenId const &tok, EUROPA::TokenSet &result) {
      if( !m_filt && tok->isActive() ) {
        EUROPA::TokenSet const &merged = tok->getMergedTokens();
        result.insert(merged.begin(), merged.end());
      }
    }
    
    bool const m_filt;
  }; // struct ::is_not_merged
  
} // ::

/*
 * class TREX::europa::details::CurrentStateId_id_traits
 */
// statics 

EUROPA::TimelineId details::CurrentStateId_id_traits::get_id(details::CurrentStateId const &cs) {
  if( cs.isId() ) 
    return cs->timeline();
  return EUROPA::TimelineId::noId();
}

/*
 * class TREX::europa::Assembly::root_tokens;
 */

void Assembly::root_tokens::notifyAdded(EUROPA::TokenId const & token) {
  if( token->master().isNoId() )
    m_roots.insert(token);  
}

void Assembly::root_tokens::notifyRemoved(EUROPA::TokenId const & token) {
  if( token->master().isNoId() ) {
    m_roots.erase(token);
    if( NULL!=m_owner )
      m_owner->discard(token);
  }
}

void Assembly::root_tokens::notifyDeactivated(EUROPA::TokenId const &token) {
  if( NULL!=m_owner )
    m_owner->cancel(token);  
}

/*
 * class TREX::europa::Assembly;
 */

// statics 

EUROPA::LabelStr const Assembly::TREX_TIMELINE("AgentTimeline");
EUROPA::LabelStr const Assembly::EXTERNAL_MODE("External");
EUROPA::LabelStr const Assembly::OBSERVE_MODE("Observe");
EUROPA::LabelStr const Assembly::INTERNAL_MODE("Internal");
EUROPA::LabelStr const Assembly::PRIVATE_MODE("Private");
EUROPA::LabelStr const Assembly::IGNORE_MODE("Ignore");
EUROPA::LabelStr const Assembly::MISSION_END("MISSION_END");
EUROPA::LabelStr const Assembly::TICK_DURATION("TICK_DURATION");
EUROPA::LabelStr const Assembly::CLOCK_VAR("AGENT_CLOCK");

std::string const Assembly::MODE_ATTR("mode");
std::string const Assembly::DEFAULT_ATTR("defaultPredicate");


std::string const Assembly::UNDEFINED_PRED("undefined");
std::string const Assembly::FAILED_PRED("Failed");

// structors

Assembly::Assembly(std::string const &name) {
  //  redirect log output to <name>.europa.log
  m_trex_schema->setStream(m_debug, name+".europa.log");
  
  // load the differrent europa modules we need
  addModule((new EUROPA::ModuleConstraintEngine())->getId());
  addModule((new EUROPA::ModuleConstraintLibrary())->getId());
  addModule((new EUROPA::ModulePlanDatabase())->getId());
  addModule((new EUROPA::ModuleRulesEngine())->getId());
  addModule((new EUROPA::ModuleTemporalNetwork())->getId());
  addModule((new EUROPA::ModuleSolvers())->getId());
  addModule((new EUROPA::ModuleNddl())->getId());

  // complete base class Initialization
  doStart();

  // gather europa entry points
  m_schema = ((EUROPA::Schema *)getComponent("Schema"))->getId();
  m_cstr_engine = ((EUROPA::ConstraintEngine *)getComponent("ConstraintEngine"))->getId();
  m_plan_db = ((EUROPA::PlanDatabase *)getComponent("PlanDatabase"))->getId();
  m_rules_engine = ((EUROPA::RulesEngine *)getComponent("RulesEngine"))->getId();

  m_roots.reset(new root_tokens(m_plan_db));
  
  // Register the new propagator used for reactor related constraints
  new ReactorPropagator(*this, EUROPA::LabelStr("trex"), m_cstr_engine);

  m_cstr_engine->setAutoPropagation(true);

  // Get extra europa extensions 
  m_trex_schema->registerComponents(*this);

  EUROPA::DomainComparator::setComparator((EUROPA::Schema *)m_schema);  
}

Assembly::~Assembly() {
  m_roots.reset();
  // cleanup base class
  doShutdown();
}

// manipulators

void Assembly::setStream() const {
  m_trex_schema->setStream(m_debug);
}

bool Assembly::playTransaction(std::string const &nddl) {
  std::string const &path = m_trex_schema->nddl_path();
  std::string ret;
  m_roots->attach(this);
  
  // configure search path for nddl
  getLanguageInterpreter("nddl")->getEngine()->getConfig()->setProperty("nddl.includePath", path);

  // parese nddl
  try {
    ret = executeScript("nddl", nddl, true);
  } catch(EUROPA::PSLanguageExceptionList const &l_err) {
    std::ostringstream err;
    err<<"Error while parsing "<<nddl<<":\n"<<l_err;
    throw EuropaException(err.str());
  } catch(Error const &error) {
    throw EuropaException("Error on parsing "+nddl+": "+error.getMsg());
  }
  if( !ret.empty() )
    throw EuropaException("Error returned after parsing "+nddl+": "+ret);

  return constraint_engine()->constraintConsistent();
}

void Assembly::configure_solvers(std::string const &cfg) {
  std::auto_ptr<EUROPA::TiXmlElement> 
    xml_cfg(EUROPA::initXml(cfg.c_str()));
  
  EUROPA::TiXmlElement 
    *filter = dynamic_cast<EUROPA::TiXmlElement *>(xml_cfg->InsertBeforeChild(xml_cfg->FirstChild(), 
  									      EUROPA::TiXmlElement("FlawFilter")));
  // Setup our deliberation solver
  filter->SetAttribute("component", TO_STRING_EVAL(TREX_DELIB_FILT));
  debugMsg("trex:init", "Load planning solver with configuration:\n"<<(*xml_cfg)); 

  m_deliberation_solver = (new EUROPA::SOLVERS::Solver(plan_db(), *xml_cfg))->getId();
  
  // Setup our synchronization solver
  filter->SetAttribute("component", TO_STRING_EVAL(TREX_SYNCH_FILT));
  
  EUROPA::TiXmlElement synch(TO_STRING_EVAL(TREX_SYNCH_MGR)),
    handler("FlawHandler");

  handler.SetAttribute("component", TO_STRING_EVAL(TREX_SYNCH_HANDLER));
  synch.InsertEndChild(handler);
  xml_cfg->InsertEndChild(synch);

  debugMsg("trex:init", "Load synchronization solver with configuration:\n"<<(*xml_cfg)); 
  m_synchronization_solver = (new EUROPA::SOLVERS::Solver(plan_db(), *xml_cfg))->getId();
}

void Assembly::init_clock_vars() {
  EUROPA::ConstrainedVariableId 
    mission_end = plan_db()->getGlobalVariable(MISSION_END),
    tick_factor = plan_db()->getGlobalVariable(TICK_DURATION);
  m_clock = plan_db()->getGlobalVariable(CLOCK_VAR);

  if( mission_end.isNoId() ) 
    throw EuropaException("Unable to find variable "+MISSION_END.toString());
  if( tick_factor.isNoId() ) 
    throw EuropaException("Unable to find variable "+TICK_DURATION.toString());
  if( m_clock.isNoId() ) 
    throw EuropaException("Unable to find variable "+CLOCK_VAR.toString());

  mission_end->restrictBaseDomain(EUROPA::IntervalIntDomain(final_tick(),
							    final_tick()));
  tick_factor->restrictBaseDomain(EUROPA::IntervalIntDomain(tick_duration(),
							    tick_duration()));
  m_clock->restrictBaseDomain(EUROPA::IntervalIntDomain(initial_tick(),
							  final_tick()));
}

void Assembly::add_state_var(EUROPA::TimelineId const &tl) {
  EUROPA::ObjectId obj(tl);
  
  if( internal(obj) || external(obj) ) {
    if( m_agent_timelines.find(tl)==m_agent_timelines.end() ) {
      details::CurrentStateId state = (new details::CurrentState(*this, tl))->getId();
      bool tl_internal = internal(obj);
      
      debugMsg("trex:timeline", "Adding State flaw manager for "<<tl->getName().toString()
	       <<"\n\t* Current T-REX mode is "<<(tl_internal?"In":"Ex")<<"ternal.");
      m_agent_timelines.insert(state);
    }
  } else {
    debugMsg("trex:always", "WARNING attempted to create a sate falw manager for non public timeline "
	     <<tl->getName().toString());
  }
}

EUROPA::TokenId Assembly::new_obs(EUROPA::ObjectId const &obj, std::string &pred, 
				  bool &undefined) {
  if( !external(obj) ) 
    throw EuropaException(obj->toString()+"is not an External timeline");
  state_iterator 
    handler = m_agent_timelines.find(EUROPA::TimelineId(obj));
  undefined = false;
    
  if( !have_predicate(obj, pred) ) {
    std::string prev = pred;
    pred = UNDEFINED_PRED;
    if( !have_predicate(obj, pred) )
      throw EuropaException("Unable to create token "+prev+" or "+UNDEFINED_PRED+
			    " for timeline "+obj->toString());
    undefined = true;
  }
  return (*handler)->new_obs(plan_db()->getClient(), pred, false);
}

void Assembly::notify(details::CurrentState const &state) {
  EUROPA::LabelStr obj_name = state.timeline()->getName();

  if( is_internal(obj_name) ) 
    notify(obj_name, state.current());
}

EUROPA::TokenId Assembly::create_token(EUROPA::ObjectId const &obj, 
				       std::string const &name,
				       bool fact) {
  EUROPA::DbClientId cli = plan_db()->getClient();
  bool rejectable = !fact;
  
  if( !schema()->isPredicate(name.c_str()) )
    throw EuropaException("Unknown predicate \""+name+"\" for object "
			  +obj->getName().toString());
  EUROPA::TokenId tok = cli->createToken(name.c_str(), NULL, rejectable, fact);
  
  if( !tok.isId() )
    throw EuropaException("Failed to create token "+name);

  // Restrict token domain to obj
  EUROPA::ConstrainedVariableId obj_var = tok->getObject();
  obj_var->specify(obj->getKey());
  
  // if( tok->isIncomplete() )
  //   tok->close();
  
  return tok;
}

void Assembly::recalled(EUROPA::TokenId const &tok) {
  // Very aggressive way to handle this 
  if( !tok->isInactive() )
    tok->cancel();
  tok->discard();
}

bool Assembly::relax(bool destructive) {
  // Clean up decisions made lastly 
  synchronizer()->reset();
  planner()->reset();
  // Remove dangling token from past decisions  
  EUROPA::TokenSet const &toks = m_roots->roots();
  EUROPA::TokenSet to_erase;
  
  for(EUROPA::TokenSet::const_iterator t=toks.begin(); toks.end()!=t; ++t) {
    bool is_past = (*t)->end()->baseDomain().getUpperBound()<=now();
    bool is_goal = (*t)->getState()->baseDomain().isMember(EUROPA::Token::REJECTED);
    
    if( is_past ) {
      if( (*t)->isFact() ) {
        if( destructive ) {
          if( !(*t)->isInactive() )
            (*t)->cancel();
          to_erase.insert(*t);
        } else if( (*t)->canBeCommitted() )
          (*t)->commit();
      } else {
        // Removing the past goals
        if( is_goal || destructive ) {
          if( !(*t)->isInactive() )
            (*t)->cancel();
          to_erase.insert(*t);
        }
      }
    } else {
      if( !( (*t)->isInactive() || (*t)->isCommitted() ) )
        (*t)->cancel();
    }
  }
  for(EUROPA::TokenSet::const_iterator t=to_erase.begin(); to_erase.end()!=t;++t)
    (*t)->discard();
  return m_cstr_engine->propagate();
}

// observers

bool Assembly::have_predicate(EUROPA::ObjectId const &obj, 
			      std::string &name) const {
  EUROPA::LabelStr obj_type = obj->getType();

  if( !schema()->isPredicate(name.c_str()) ) {
    std::string long_name = obj_type.toString()+"."+name;
    if( schema()->isPredicate(long_name.c_str()) )
      name = long_name;
    else 
      return false;
  } 
  return schema()->canBeAssigned(obj_type, name.c_str());
}


bool Assembly::internal(EUROPA::TokenId const &tok) const {
  EUROPA::ObjectDomain const &dom = tok->getObject()->lastDomain();
  std::list<EUROPA::ObjectId> objs = dom.makeObjectList();
  
  for(std::list<EUROPA::ObjectId>::const_iterator o=objs.begin();
      objs.end()!=o; ++o) 
    if( !internal(*o) )
      return false;
  return true;
}

bool Assembly::internal(details::CurrentState const &state) const {
  return is_internal(state.timeline()->getName());
}

bool Assembly::internal(EUROPA::ObjectId const &obj) const {
  return is_agent_timeline(obj) && is_internal(obj->getName());
}

bool Assembly::external(EUROPA::TokenId const &tok) const {
  EUROPA::ObjectDomain const &dom = tok->getObject()->lastDomain();
  std::list<EUROPA::ObjectId> objs = dom.makeObjectList();
  
  for(std::list<EUROPA::ObjectId>::const_iterator o=objs.begin();
      objs.end()!=o; ++o) 
    if( !external(*o) )
      return false;
  return true;
}

bool Assembly::external(EUROPA::ObjectId const &obj) const {
  return is_agent_timeline(obj) && is_external(obj->getName());
}

bool Assembly::ignored(EUROPA::TokenId const &tok) const {
  EUROPA::ObjectDomain const &dom = tok->getObject()->lastDomain();
  std::list<EUROPA::ObjectId> objs = dom.makeObjectList();
  
  for(std::list<EUROPA::ObjectId>::const_iterator o=objs.begin();
      objs.end()!=o; ++o) 
    if( !ignored(*o) )
      return false;
  return !objs.empty();
}

EUROPA::ConstrainedVariableId Assembly::attribute(EUROPA::ObjectId const &obj,
						  std::string const &attr) const {
  std::string full_name = obj->getName().toString()+"."+attr;
  EUROPA::ConstrainedVariableId var = obj->getVariable(full_name);
  if( var.isNoId() )
    throw EuropaException("Variable \""+full_name+"\" does not exist.");
  return var;
}

void Assembly::print_plan(std::ostream &out, bool expanded) const {
  EUROPA::TokenSet const tokens = plan_db()->getTokens();
  is_not_merged filter(expanded);
  
  out<<"digraph plan_"<<now()<<" {\n"
     <<"  node[shape=\"box\"];\n\n";
  boost::filter_iterator<is_not_merged, EUROPA::TokenSet::const_iterator>
    it(filter, tokens.begin(), tokens.end()), 
    endi(filter, tokens.end(), tokens.end());
  // Iterate through plan tokens
  for( ; endi!=it; ++it) {
    EUROPA::eint key = (*it)->getKey();
    // display the token as a node
    out<<"  t"<<key<<"[label=\""<<(*it)->getPredicateName().toString()
       <<'('<<key<<") {\\n";
    if( (*it)->isIncomplete() ) 
      out<<"incomplete\\n";
    //    if( (*it)->isCommitted() ) 
    //   out<<"commit\\n";
    std::vector<EUROPA::ConstrainedVariableId> const &vars = (*it)->getVariables();
    for(std::vector<EUROPA::ConstrainedVariableId>::const_iterator v=vars.begin();
        vars.end()!=v; ++v) {
      // print all token attributes
      out<<"  "<<(*v)->getName().toString()<<'='<<(*v)->toString()<<"\\n";
    }
    out<<"}\"";
    if( ignored(*it) )
      out<<" color=grey"; // ignored tokens are greyed
    else if( filter.is_fact(*it) )
      out<<" color=red"; // fact tokens are red
    std::ostringstream styles;
    bool comma = false;
    
    if( filter.is_goal(*it) ) {
      styles<<"rounded"; // goal have rounded corner
      comma=true;
    }
    if( (*it)->isCommitted() ) {
      if( comma )
        styles<<',';
      styles<<"dashed"; // commits are dashed
      comma = true;
    }
    if( comma )
      out<<" style=\""<<styles.str()<<"\" "; // display style modifiers
    out<<"];\n";
    if( (*it)->isMerged() ) {
      EUROPA::eint active = (*it)->getActiveToken()->getKey();
      // connect the merged token to its active counterpart
      out<<"  t"<<key<<"->t"<<active<<"[color=grey];\n"; 
    }
    EUROPA::TokenSet toks;
    toks.insert(*it);
    filter.merged(*it, toks);
    // display the relation to the master token(s)
    for(EUROPA::TokenSet::const_iterator t=toks.begin(); toks.end()!=t; ++t) {
      EUROPA::TokenId master = (*t)->master();
      
      if( master.isId() ) {
        if( expanded ) 
          key = (*t)->getKey();
        out<<"  t"<<master->getKey()<<"->t"<<key
           <<"[label=\""<<(*t)->getRelation().toString()<<"\"";
        if( (*it)!=(*t) )
          out<<" color=grey";
        out<<"];\n";
      }
    }
  }
  out<<"}"<<std::endl;
}


