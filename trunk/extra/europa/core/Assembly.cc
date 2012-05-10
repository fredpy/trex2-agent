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
#include "trex/europa/Assembly.hh"
#include "trex/europa/bits/europa_helpers.hh"
#include "private/Schema.hh"
#include "private/CurrentState.hh"

#include "trex/europa/ReactorPropagator.hh"
#include "trex/europa/DeliberationFilter.hh"
#include "trex/europa/SynchronizationManager.hh"

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
 * class TREX:europa::Assembly
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
std::string const Assembly::PLAN_ATTR("with_plan");

std::string const Assembly::UNDEFINED_PRED("undefined");
std::string const Assembly::FAILED_PRED("Failed");

// structors

Assembly::Assembly(std::string const &name):m_name(name) {
  //  redirect log output to <name>.europa.log
  m_trex_schema->setStream(m_debug, m_name+".europa.log");

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
  m_plan = ((EUROPA::PlanDatabase *)getComponent("PlanDatabase"))->getId();

  m_proxy.reset(new listener_proxy(*this));

  // Register the new propagator used for reactor related constraints
  new ReactorPropagator(*this, EUROPA::LabelStr("trex"), m_cstr_engine);

  m_cstr_engine->setAutoPropagation(true);

  // Get extra europa extensions
  m_trex_schema->registerComponents(*this);

  EUROPA::DomainComparator::setComparator((EUROPA::Schema *)m_schema);
}

Assembly::~Assembly() {
  setStream();
  debugMsg("trex:end", "Destroying "<<m_name);
  m_proxy.reset();
  // cleanup base class
  doShutdown();
}

// modifiers

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
    debugMsg("trex:always", "WARNING attempted to create a sate flaw manager for non public timeline "
	     <<tl->getName().toString());
  }
}

void Assembly::new_tick() {
  debugMsg("trex:tick", "Updating clock to ["<<now()<<", "<<final_tick()<<"]\n");
  m_clock->restrictBaseDomain(EUROPA::IntervalIntDomain(now(), final_tick()));

  debugMsg("trex:tick", "Updating non-started goals to start after "<<now());
  boost::filter_iterator<details::is_rejectable, EUROPA::TokenSet::const_iterator>
    t(m_roots.begin(), m_roots.end()), end_t(m_roots.end(), m_roots.end());
  EUROPA::IntervalIntDomain future(now(),
                                   std::numeric_limits<EUROPA::eint>::infinity());

  for(; end_t!=t; ++t) {
    EUROPA::TokenId active = (*t);

    if( (*t)->isMerged() )
      active = (*t)->getActiveToken();

    if( now()<=active->start()->lastDomain().getUpperBound() ) {
      debugMsg("trex:tick", "Before: "<<(*t)->toString()<<".start="<<active->start()->lastDomain().toString());
      details::restrict_base(*t, (*t)->start(), future);
      details::restrict_base(*t, (*t)->end(), EUROPA::IntervalIntDomain(now()+1,
                                                                        std::numeric_limits<EUROPA::eint>::infinity()));
      debugMsg("trex:tick", "After: "<<(*t)->toString()<<".start="<<active->start()->lastDomain().toString());
    } else if( now()<=active->end()->lastDomain().getUpperBound() ) {
      debugMsg("trex:tick", "Before: "<<(*t)->toString()<<".end="<<active->end()->lastDomain().toString());
      details::restrict_base(*t, (*t)->end(), future);
      debugMsg("trex:tick", "After: "<<(*t)->toString()<<".end="<<active->end()->lastDomain().toString());
    }
  }
}

void Assembly::terminate(EUROPA::TokenId const &tok) {
  details::is_rejectable rejectable;
  EUROPA::TokenId active = tok;

  m_completed.insert(tok);

  // I assume here that if a goals ends time iis in the past then the goal is completed

  if( tok->isMerged() ) {
    active = tok->getActiveToken();
    if( rejectable(active) && discard(active) ) {
      details::restrict_base(active, active->end(), tok->end()->baseDomain());
      details::restrict_base(active, active->start(), tok->start()->baseDomain());
      m_completed.insert(active);
    }
  }
  for(EUROPA::TokenSet::const_iterator i=active->getMergedTokens().begin();
      active->getMergedTokens().end()!=i; ++i) {
    if( rejectable(*i) && discard(*i) ) {
      details::restrict_base(*i, (*i)->end(), tok->end()->baseDomain());
      details::restrict_base(*i, (*i)->start(), tok->start()->baseDomain());
      m_completed.insert(*i);
    }
  }
}

// manipulators

void Assembly::setStream() const {
  m_trex_schema->setStream(m_debug);
}

bool Assembly::playTransaction(std::string const &nddl) {
  std::string const &path = m_trex_schema->nddl_path();
  std::string ret;

  // configure search path for nddl
  getLanguageInterpreter("nddl")->getEngine()->getConfig()->setProperty("nddl.includePath", path);

  // parse nddl
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

  m_planner = (new EUROPA::SOLVERS::Solver(plan_db(), *xml_cfg))->getId();

  // Setup our synchronization solver
  filter->SetAttribute("component", TO_STRING_EVAL(TREX_SYNCH_FILT));

  EUROPA::TiXmlElement synch(TO_STRING_EVAL(TREX_SYNCH_MGR)),
  handler("FlawHandler");

  handler.SetAttribute("component", TO_STRING_EVAL(TREX_SYNCH_HANDLER));
  synch.InsertEndChild(handler);
  xml_cfg->InsertEndChild(synch);

  debugMsg("trex:init", "Load synchronization solver with configuration:\n"<<(*xml_cfg));
  m_synchronizer = (new EUROPA::SOLVERS::Solver(plan_db(), *xml_cfg))->getId();
}

void Assembly::notify(details::CurrentState const &state) {
  EUROPA::LabelStr obj_name = state.timeline()->getName();

  if( is_internal(obj_name) ) {
    debugMsg("trex:notify", "Post observation "<<state.timeline()->toString()
             <<'.'<<state.current()->getUnqualifiedPredicateName().toString()<<":\n"
             <<state.current()->toLongString());
    notify(obj_name, state.current());
  }
}

bool Assembly::do_synchronize() {
  // 1) apply external observations
  for(external_iterator i=begin_external(); end_external()!=i; ++i) {
    if( !(*i)->commit() ) {
      debugMsg("trex:synch", "Failed to integrate external state of "<<(*i)->timeline()->toString());
      return false;
    }
  }

  // 2) execute synchronizer
  if( !synchronizer()->solve() ) {
    debugMsg("trex:synch", "Failed to resolve synchronization for tick "<<now());
    return false;
  }

  // 3) apply internal observations
  for(internal_iterator i=begin_internal(); end_internal()!=i; ++i)
    (*i)->commit(); // It was consistent from synchronizer => never fail

  return true;
}

bool Assembly::relax(bool aggressive) {
  details::is_rejectable rejectable;
  EUROPA::TokenSet to_erase;

  // Clean up decisions made by solvers
  synchronizer()->reset();
  planner()->reset();

  for(EUROPA::TokenSet::const_iterator t=m_roots.begin(); m_roots.end()!=t; ++t) {
    bool is_past = (*t)->end()->baseDomain().getUpperBound()<=now();

    if( is_past ) {
      if( (*t)->isFact() ) {
        if( aggressive ) {
          if( !(*t)->isInactive() )
            plan_db()->getClient()->cancel(*t);
          to_erase.insert(*t);
        }
      } else {
        if( rejectable(*t) || aggressive ) {
          if( !(*t)->isInactive() )
            plan_db()->getClient()->cancel(*t);
          to_erase.insert(*t);
        }
      }
    } else {
      if( !( (*t)->isInactive() || (*t)->isFact() ) )
        plan_db()->getClient()->cancel(*t);
    }
  }

  for(EUROPA::TokenSet::const_iterator t=to_erase.begin(); to_erase.end()!=t;++t)
    plan_db()->getClient()->deleteToken(*t);

  return constraint_engine()->propagate();
}

void Assembly::archive() {
  plan_db()->archive(now()-1);
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

  return tok;
}

EUROPA::TokenId Assembly::new_obs(EUROPA::ObjectId const &obj,
                                  std::string &pred,
                                  bool &undefined) {
  if( !external(obj) )
    throw EuropaException(obj->toString()+"is not an External timeline");
  state_iterator handler = m_agent_timelines.find(EUROPA::TimelineId(obj));

  undefined = false;

  if( !have_predicate(obj, pred) ) {
    std::string prev = pred;
    pred = UNDEFINED_PRED;
    if( !have_predicate(obj, pred) )
      throw EuropaException("Unable to create token "+prev+" or "+UNDEFINED_PRED+
			    " for timeline "+obj->toString());
    undefined = true;
  }
  return (*handler)->new_obs(pred, false);
}

void Assembly::recalled(EUROPA::TokenId const &tok) {
  // Very aggressive way to handle this
  debugMsg("trex:recall", "Goal "<<tok->toString()<<" recalled.");

  if( !tok->isInactive() ) {
    debugMsg("trex:recall", "Cancelling "<<tok->toString());
    plan_db()->getClient()->cancel(tok);
  }
  debugMsg("trex:recall", "Destroying "<<tok->toString());
  plan_db()->getClient()->deleteToken(tok);
}


// observers

bool Assembly::with_plan(EUROPA::ObjectId const &obj) const {
  if( is_agent_timeline(obj) ) {
    EUROPA::ConstrainedVariableId var = attribute(obj, PLAN_ATTR);
    if( var.isId() && var->lastDomain().isSingleton() )
      return var->lastDomain().getSingletonValue()!=0.0;
  }
  return false;
}

bool Assembly::is_goal(EUROPA::TokenId const &tok) const {
  EUROPA::StateDomain const &state = tok->getState()->baseDomain();
  return state.isMember(EUROPA::Token::REJECTED) && internal(tok);
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
  return !objs.empty(); // we do not ignore tokens with no possible object
}

bool Assembly::is_agent_timeline(EUROPA::TokenId const &token) const {
  return schema()->isA(token->getBaseObjectType(), TREX_TIMELINE);
}



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
#ifdef EUROPA_HAVE_EFFECT
    out<<"type: ";
    if( (*it)->hasAttributes(EUROPA::PSTokenType::ACTION) ) 
      out<<"ACTION";
    else if( (*it)->hasAttributes(EUROPA::PSTokenType::PREDICATE) )
      out<<"PREDICATE";
    else 
      out<<"???";
    out<<"\\n";
#endif // EUROPA_HAVE_EFFECT
    std::vector<EUROPA::ConstrainedVariableId> const &vars = (*it)->getVariables();
    for(std::vector<EUROPA::ConstrainedVariableId>::const_iterator v=vars.begin();
        vars.end()!=v; ++v) {
      // print all token attributes
      out<<"  "<<(*v)->getName().toString()<<'='<<(*v)->toString()<<"\\n";
    }
    if( (*it)->isActive() && !expanded ) {
      EUROPA::TokenSet const &merged = (*it)->getMergedTokens();
      if( !merged.empty() ) {
        out<<"merged={";
        EUROPA::TokenSet::const_iterator m = merged.begin();
        out<<(*(m++))->getKey();
        for(; merged.end()!=m; ++m)
          out<<", "<<(*m)->getKey();
        out<<"}\\n";
      }
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
#ifdef EUROPA_HAVE_EFFECT
    if( (*it)->hasAttributes(EUROPA::PSTokenType::ACTION) ) {
      if( comma )
        styles.put(',');
      else
        comma = true;
      styles<<"filled";
    }
#endif // EUROPA_HAVE_EFFECT
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
           <<"[label=\""<<(*t)->getRelation().toString();
#ifdef EUROPA_HAVE_EFFECT
        if( (*t)->hasAttributes(EUROPA::PSTokenType::EFFECT) )
          out<<"\\n(effect)";
        if( (*t)->hasAttributes(EUROPA::PSTokenType::CONDITION) )
          out<<"\\n(condition)";
#endif // EUROPA_HAVE_EFFECT
        out<<"\"";
        if( (*it)!=(*t) )
          out<<" color=grey";
        out<<"];\n";
      }
    }
  }
  out<<"}"<<std::endl;
}

void Assembly::getFuturePlan()
{
    internal_iterator it = begin_internal(), end = end_internal();
    for(; it!=end; ++it)
    {
        EUROPA::TimelineId timeline((*it)->timeline());
        const std::list<EUROPA::TokenId>& tokens = timeline->getTokenSequence();
        for(std::list<EUROPA::TokenId>::const_iterator token = tokens.begin();
        token!=tokens.end(); ++token)
        {
             if((*token)->end()->lastDomain().getLowerBound()>now()+1)
             {
                 plan_dispatch(timeline,*token);
             }
        }
    }
}

/*
 * class TREX::europa::Assembly::listener_proxy
 */

void Assembly::listener_proxy::notifyAdded(EUROPA::TokenId const &token) {
  EUROPA::TokenId master = token->master();

  if( master.isNoId() ) {
    m_owner.m_roots.insert(token);
  }
}

void Assembly::listener_proxy::notifyRemoved(EUROPA::TokenId const &token) {
  m_owner.m_roots.erase(token);
  m_owner.m_completed.erase(token);

  if( m_owner.is_agent_timeline(token) ) {
    if( token->isFact() ) {
      // Not very efficient bu  it should do for now ...
      for (Assembly::state_iterator i=m_owner.m_agent_timelines.begin();
           m_owner.m_agent_timelines.end()!=i; ++i)
        (*i)->erased(token);
    }
    m_owner.discard(token);
  }
}

void Assembly::listener_proxy::notifyActivated(EUROPA::TokenId const &token) {

}

void Assembly::listener_proxy::notifyDeactivated(EUROPA::TokenId const &token) {
  if( m_owner.is_agent_timeline(token) )
    m_owner.cancel(token);
}

void Assembly::listener_proxy::notifyMerged(EUROPA::TokenId const &token) {

}

void Assembly::listener_proxy::notifySplit(EUROPA::TokenId const &token) {

}

void Assembly::listener_proxy::notifyRejected(EUROPA::TokenId const &token) {

}

void Assembly::listener_proxy::notifyReinstated(EUROPA::TokenId const &token) {

}

void Assembly::listener_proxy::notifyCommitted(EUROPA::TokenId const &token) {

}

void Assembly::listener_proxy::notifyTerminated(EUROPA::TokenId const &token) {

}


/*

// statics

EUROPA::TimelineId details::CurrentStateId_id_traits::get_id(details::CurrentStateId const &cs) {
  if( cs.isId() )
    return cs->timeline();
  return EUROPA::TimelineId::noId();
}

EUROPA::eint details::token_id_traits::get_id(EUROPA::TokenId const &t) {
  return t->getKey();
}



void Assembly::root_tokens::notifyAdded(EUROPA::TokenId const & token) {
  if( token->master().isNoId() )
    m_roots.insert(token);
}

void Assembly::root_tokens::notifyRemoved(EUROPA::TokenId const & token) {
  m_committed.erase(token);
  if( token->master().isNoId() ) {
    m_roots.erase(token);
    if( NULL!=m_owner )
      m_owner->doDiscard(token);
  }
}

void Assembly::root_tokens::notifyDeactivated(EUROPA::TokenId const &token) {
  if( NULL!=m_owner )
    m_owner->cancel(token);
}

void Assembly::root_tokens::notifyCommitted(EUROPA::TokenId const & token) {
  m_committed.insert(token);
  m_roots.erase(token);
}



// manipulators


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

void Assembly::doDiscard(EUROPA::TokenId const &tok) {
  m_terminated.erase(tok->getKey());
  discard(tok);
}


void Assembly::recalled(EUROPA::TokenId const &tok) {
  // Very aggressive way to handle this
  if( !tok->isInactive() )
    tok->cancel();
  tok->discard();
}

void Assembly::terminate(EUROPA::TokenId const &token) {
  if( m_terminated.insert(token).second ) {
    debugMsg("trex:token", "Terminated token "<<token->toString());
  }
//  if( token->canBeCommitted() )
//    token->commit();
}


void Assembly::archive() {
  EUROPA::DbClientId cli = plan_db()->getClient();
  EUROPA::TokenSet to_del;

  debugMsg("trex:archive", '['<<now()<<"] ============= START archiving ============");

  // Process first the already committed tokens
  EUROPA::TokenSet::const_iterator t;
  for(t=m_roots->committed().begin(); m_roots->committed().end()!=t; ++t) {
    bool can_delete = true;
    debugMsg("trex:archive", "Evaluating committed token "<<(*t)->toString());

    EUROPA::TokenSet merged_t = (*t)->getMergedTokens();

    for(EUROPA::TokenSet::const_iterator i=merged_t.begin();
        merged_t.end()!=i; ++i) {
      EUROPA::TokenId master = (*i)->master();
      if( master.isId() ) {
        if( master->isCommitted() ) {
          std::vector<EUROPA::ConstrainedVariableId> const &actives = (*t)->getVariables();
          std::vector<EUROPA::ConstrainedVariableId> const &mergeds = (*i)->getVariables();
          // constraint_engine()->propagate();
          for(size_t v=1; v<actives.size(); ++v)
            actives[v]->restrictBaseDomain(mergeds[v]->lastDomain());
          debugMsg("trex:archive", "Removing merged token "<<(*i)->toString()
                   <<':'<<(*i)->start()->lastDomain().toString()<<"->"<<(*i)->end()->lastDomain().toString());
          cli->cancel(*i);
          to_del.insert(*i);
        } else {
          debugMsg("trex:archive", "Cannot cancel  "<<(*t)->toString()
                   <<":\n\t"<<(*i)->toString()<<"'s master is not commmitted");
          can_delete=false;
        }
      } else {
        if( (*i)->isFact() ) {
          debugMsg("trex:archive", "Collapsing committed token "<<(*t)->toString()
                   <<" with fact "<<(*i)->toString());
          (*t)->makeFact();
          to_del.insert(*i);
	  for(state_map::const_iterator j=m_agent_timelines.begin();
	      m_agent_timelines.end()!=j; ++j)
	    (*j)->replaced(*i);
        } else {
          debugMsg("trex:archive", "Cannot cancel  "<<(*t)->toString()
                   <<":\n\t non factual merged token "<<(*i)->toString()<<" is in the way");
          can_delete = false;
        }
      }
    }

    EUROPA::TokenSet slaves_t = (*t)->slaves();
    for(EUROPA::TokenSet::const_iterator i=slaves_t.begin();
        slaves_t.end()!=i; ++i) {

      if( (*i)->isInactive() ) {
        debugMsg("trex:archive", "Removing inactive slave "<<(*i)->toString()
                 <<':'<<(*i)->start()->lastDomain().toString()<<"->"<<(*i)->end()->lastDomain().toString());
        to_del.insert(*i);
      } else {
        bool ended;


        if( (*i)->isActive() ) {
          if( (*i)->isCommitted() ) {
            (*i)->removeMaster(*t);
            debugMsg("trex:archive", "Disconnect comitted token "<<(*i)->toString()
                     <<" from its master");
            ended = true;
          } else {
            ended = (*i)->end()->lastDomain().getUpperBound()<now();
            can_delete = false;
          }
        } else {
          details::restrict_bases(*i);

          ended = (*i)->end()->baseDomain().getUpperBound()<now();
          can_delete = false;
        }
        if( ended && !(*i)->isCommitted() ) {
          m_terminated.insert(*i);
          debugMsg("trex:archive", "Mark "<<(*i)->toString()
                   <<" as finished");
        }
      }
    }
    if( can_delete ) {
      debugMsg("trex:archive", "Removing committed token "<<(*t)->toString()
               <<':'<<(*t)->start()->lastDomain().toString()<<"->"<<(*t)->end()->lastDomain().toString());
      to_del.insert(*t);
    }
  }
  debugMsg("trex:archive", "Deleting "<<to_del.size()<<" token(s).");
  for(t=to_del.begin(); to_del.end()!=t; ++t) {
    if( !(t->isInvalid() || (*t)->isDeleted()) ) {
      m_terminated.erase((*t)->getKey());
      if( !(*t)->isInactive() )
        cli->cancel(*t);
      debugMsg("trex:archive", "Discard "<<(*t)->toString());
      (*t)->discard();
    }
  }
  to_del.clear();

  debugMsg("trex:archive", "Looking for terminated token(s).");
  for(token_set::const_iterator t=m_terminated.begin(); m_terminated.end()!=t; ++t) {
    EUROPA::TokenId tok(*t);

    if( tok->isInactive() ) {
      debugMsg("trex:archive", "Removing inactive past token "<<(*t)->toString());
      to_del.insert(*t); // May need some extra guard here
    } else if( tok->isMerged() ) {
      EUROPA::TokenId active = tok->getActiveToken();
      if( active->isCommitted() ) {
        std::vector<EUROPA::ConstrainedVariableId> const &actives = active->getVariables();
        std::vector<EUROPA::ConstrainedVariableId> const &mergeds = tok->getVariables();
        debugMsg("trex:archive", "Removing merged token "<<(*t)->toString());
        for(size_t v=1; v<actives.size(); ++v)
          actives[v]->restrictBaseDomain(mergeds[v]->lastDomain());
        cli->cancel(tok);
        to_del.insert(tok);
      } else
        m_terminated.insert(active);
      continue;
    }
    if(tok->canBeCommitted() ) {
      debugMsg("trex:archive", "Mark "<<tok->toString()<<" as commmitted.");
      tok->commit();
    }
  }
  debugMsg("trex:archive", "Deleting "<<to_del.size()<<" token(s).");
  for(t=to_del.begin(); to_del.end()!=t; ++t) {
    if( !(t->isInvalid() || (*t)->isDeleted()) ) {
      m_terminated.erase((*t)->getKey());
      if( !(*t)->isInactive() )
        cli->cancel(*t);
      debugMsg("trex:archive", "Discard "<<(*t)->toString());
      (*t)->discard();
    }
  }
  to_del.clear();

  constraint_engine()->propagate();
  debugMsg("trex:archive", '['<<now()<<"] ============= END archiving ============");

  // plan_db()->archive(now()-1);
}

void Assembly::new_tick() {
  EUROPA::TokenSet::const_iterator t;
  for(t=m_roots->roots().begin(); m_roots->roots().end()!=t; ++t) {
    if( (*t)->getState()->baseDomain().isMember(EUROPA::Token::REJECTED) ) {
      EUROPA::TokenId active = (*t);
      if( (*t)->isMerged() )
	active = (*t)->getActiveToken();
      if( now()<=active->start()->lastDomain().getUpperBound() ) {
	EUROPA::IntervalIntDomain
	  new_start(now(), std::numeric_limits<EUROPA::eint>::infinity());
	details::restrict_base(*t, (*t)->start(), new_start);
      }
    }
  }
}

// observers



*/

