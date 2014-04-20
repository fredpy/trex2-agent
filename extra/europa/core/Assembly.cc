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

// include plasma header as system files in order to disable warnings
# define TREX_PP_SYSTEM_FILE <PLASMA/ModuleConstraintEngine.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/ModulePlanDatabase.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/ModuleRulesEngine.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/ModuleTemporalNetwork.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/ModuleSolvers.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/ModuleNddl.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/RulesEngine.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/Propagators.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/Schema.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/NddlInterpreter.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/Timeline.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/TokenVariable.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/PlanDatabaseWriter.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/Context.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/XMLUtils.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/Debug.hh>
# include <trex/europa/bits/system_header.hh>

#include <boost/scope_exit.hpp>

#include <algorithm>
#include <iterator>


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

EUROPA::eint details::CurrentStateId_id_traits::get_id(details::CurrentStateId const &cs) {
  if( cs.isId() )
    return cs->timeline()->getKey();
  return std::numeric_limits<EUROPA::eint>::minus_infinity();
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
EUROPA::LabelStr const Assembly::MISSION_START("MISSION_START");
EUROPA::LabelStr const Assembly::TICK_DURATION("TICK_DURATION");
EUROPA::LabelStr const Assembly::CLOCK_VAR("AGENT_CLOCK");

std::string const Assembly::MODE_ATTR("mode");
std::string const Assembly::DEFAULT_ATTR("defaultPredicate");
std::string const Assembly::PLAN_ATTR("with_plan");

std::string const Assembly::UNDEFINED_PRED("undefined");
std::string const Assembly::FAILED_PRED("Failed");

bool Assembly::actions_supported() {
#ifdef EUROPA_HAVE_EFFECT
  return true;
#else
  return false;
#endif
}


// structors

Assembly::Assembly(std::string const &name, size_t steps,
                   size_t depth)
  :m_in_synchronization(false), m_name(name),
   m_debug_file(m_trex_schema->service()),
   m_synchSteps(steps), m_synchDepth(depth),
   m_archiving(false) {
     m_debug_file.open(m_trex_schema->file_name(m_name+"/europa.log"));
     m_debug.open(utils::async_buffer_sink(m_debug_file));
     m_trex_schema->setStream(m_debug);
     
  //  redirect log output to <name>/europa.log
  //  m_trex_schema->setStream(m_debug, m_name+"/europa.log");
  if( m_synchSteps==0 )
    m_synchSteps = std::numeric_limits<unsigned int>::max();
  else
    debugMsg("trex:always", "Limiting synchronization to "
             <<m_synchSteps<<" steps");
  if( m_synchDepth==0 )
    m_synchDepth = m_synchSteps;
  else if( m_synchDepth<m_synchSteps )
    debugMsg("trex:always", "Limiting synchronization depth to "
             <<m_synchDepth);


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


  // Register the new propagator used for reactor related constraints
  new ReactorPropagator(*this, EUROPA::LabelStr("trex"), m_cstr_engine);

  m_cstr_engine->setAutoPropagation(false);

  // Get extra europa extensions
  m_trex_schema->registerComponents(*this);

  m_synchListener.reset(new synchronization_listener(*this));
  m_ce_listener.reset(new ce_listener(*this));
  m_proxy.reset(new listener_proxy(*this));

  EUROPA::DomainComparator::setComparator((EUROPA::Schema *)m_schema);
}

Assembly::~Assembly() {
  setStream();
  // debugMsg("trex:end", "Destroying "<<m_name);
  m_proxy.reset();
  m_ce_listener.reset();
  m_synchListener.reset();
  // cleanup base class
  doShutdown();
}

// modifiers

void Assembly::init_clock_vars() {
  EUROPA::ConstrainedVariableId
    mission_end = plan_db()->getGlobalVariable(MISSION_END),
    mission_start = plan_db()->getGlobalVariable(MISSION_START),
    tick_factor = plan_db()->getGlobalVariable(TICK_DURATION);

  m_clock = plan_db()->getGlobalVariable(CLOCK_VAR);

  if( mission_end.isNoId() )
    throw EuropaException("Unable to find variable "+MISSION_END.toString());
  if( mission_start.isNoId() )
    throw EuropaException("Unable to find variable "+MISSION_START.toString());
  if( tick_factor.isNoId() )
    throw EuropaException("Unable to find variable "+TICK_DURATION.toString());
  if( m_clock.isNoId() )
    throw EuropaException("Unable to find variable "+CLOCK_VAR.toString());

  mission_start->restrictBaseDomain(EUROPA::IntervalIntDomain(initial_tick(),
							      initial_tick()));
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
    if( m_agent_timelines.find(tl->getKey())==m_agent_timelines.end() ) {
      details::CurrentStateId
        state = (new details::CurrentState(*this, tl))->getId();
      bool tl_internal = internal(obj);

      debugMsg("trex:timeline", "Adding State flaw manager for "
               <<tl->getName().toString()
	       <<"\n\t* Current T-REX mode is "
               <<(tl_internal?"In":"Ex")<<"ternal.");
      m_agent_timelines.insert(state);
    }
  } else {
    debugMsg("trex:always", "WARNING attempted to create a state"
             <<" flaw manager for non public timeline "
	     <<tl->getName().toString());
  }
}

void Assembly::new_tick() {
  bool auto_prop = constraint_engine()->getAutoPropagation();
  constraint_engine()->setAutoPropagation(false);
  
  debugMsg("trex:tick", "START new_tick["<<now()<< "]-----------------------------------------------------");
  debugMsg("trex:tick", "Updating clock to ["<<now()<<", "<<final_tick()<<"]");
  m_clock->restrictBaseDomain(EUROPA::IntervalIntDomain(now(), final_tick()));

  debugMsg("trex:tick", "Updating non-started goals to start after "<<now());
  boost::filter_iterator<details::is_rejectable,
                         EUROPA::TokenSet::const_iterator>
    t(m_roots.begin(), m_roots.end()), end_t(m_roots.end(), m_roots.end());
  EUROPA::IntervalIntDomain future(now(),
                                   std::numeric_limits<EUROPA::eint>::infinity());
  
  
  for(; end_t!=t; ++t) {
    EUROPA::TokenId active = (*t);

    if( (*t)->isMerged() )
      active = (*t)->getActiveToken();

    debugMsg("trex:tick", "Checking start & end times of "
             <<(*t)->getUnqualifiedPredicateName().toString()
             <<'('<<(*t)->getKey()<<") : "
             <<active->start()->lastDomain().toString()
             <<" -> "<<active->end()->lastDomain().toString());
    
    if( now()<=active->start()->lastDomain().getUpperBound() ) {
      debugMsg("trex:tick", "Maintaining token start time to be after "<<now());
      debugMsg("trex:tick", "Before: "<<(*t)->toString()<<".start="<<(*t)->start()->baseDomain().toString()<<" * "<<active->start()->lastDomain().toString());
      details::restrict_base(*t, (*t)->start(), future);
      details::restrict_base(*t, (*t)->end(), EUROPA::IntervalIntDomain(now()+1,
                                                                        std::numeric_limits<EUROPA::eint>::infinity()));
      debugMsg("trex:tick", "After: "<<(*t)->toString()<<".start="<<(*t)->start()->baseDomain().toString()<<" * "<<active->start()->lastDomain().toString());
    } else if( now()<=active->end()->lastDomain().getUpperBound() ) {
      debugMsg("trex:tick", "Maintaining token end time to be after "<<now());
      debugMsg("trex:tick", "Before: "<<(*t)->toString()<<".end="<<(*t)->end()->baseDomain().toString()<<" * "<<active->end()->lastDomain().toString());
      details::restrict_base(*t, (*t)->end(), future);
      debugMsg("trex:tick", "After: "<<(*t)->toString()<<".end="<<(*t)->end()->baseDomain().toString()<<" * "<<active->end()->lastDomain().toString());
    }
  }
  debugMsg("trex:tick", "END new_tick["<<now()<< "]-----------------------------------------------------");
  // constraint_engine()->propagate();
  constraint_engine()->setAutoPropagation(auto_prop);
}

void Assembly::terminate(EUROPA::TokenId const &tok) {
  details::is_rejectable rejectable;
  EUROPA::TokenId active = tok;

  if( !tok->isCommitted() &&
     m_committed.find(tok)==m_committed.end() ) {
    if( m_completed.insert(tok).second )
      m_updated_commit = true;
  }

  details::restrict_base(tok, tok->end(),
                         details::active(tok)->end()->lastDomain());
  // I assume here that if a goals ends time is in the past then the goal is completed

  if( tok->isMerged() ) {
    active = tok->getActiveToken();
    if( rejectable(active) && discard(active) ) {
      details::restrict_base(active, active->end(), tok->end()->baseDomain());
      details::restrict_base(active, active->start(), tok->start()->baseDomain());
      if( !active->isCommitted() )
        if( m_completed.insert(active).second )
          m_updated_commit = true;
    }
  } else if( tok->isActive() )
    discard(tok);
  for(EUROPA::TokenSet::const_iterator i=active->getMergedTokens().begin();
      active->getMergedTokens().end()!=i; ++i) {
    if( rejectable(*i) && discard(*i) ) {
      details::restrict_base(*i, (*i)->end(), tok->end()->baseDomain());
      details::restrict_base(*i, (*i)->start(), tok->start()->baseDomain());
      if( !(*i)->isCommitted() )
        if( m_completed.insert(*i).second )
          m_updated_commit = true;
    }
  }
}

// manipulators

void Assembly::setStream() const {
  m_trex_schema->setStream(m_debug);
}

bool Assembly::locate_nddl(std::string &nddl) const {
  bool found;
  nddl = m_trex_schema->use(nddl, found);
  return found;
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

void Assembly::configure_solvers(std::string const &synchronizer,
				 std::string const &planner) {
  debugMsg("trex:init", "Loading planner configuration from \""<<planner<<"\".");
  UNIQ_PTR<EUROPA::TiXmlElement>
    xml_cfg(EUROPA::initXml(planner.c_str()));

  debugMsg("trex:init", "Injecting global filter for planning horizon");
  EUROPA::TiXmlElement
    *filter=dynamic_cast<EUROPA::TiXmlElement *>(xml_cfg->InsertBeforeChild(xml_cfg->FirstChild(),
									    EUROPA::TiXmlElement("FlawFilter")));

  filter->SetAttribute("component", TO_STRING_EVAL(TREX_DELIB_FILT));
  debugMsg("trex:init", "Configuring europa planner with xml:\n"<<(*xml_cfg));
  m_planner = (new EUROPA::SOLVERS::Solver(plan_db(), *xml_cfg))->getId();

  if( synchronizer!=planner ) {
    debugMsg("trex:init", "Loading synchronizer configuration from \""<<planner<<"\".");
    xml_cfg.reset(EUROPA::initXml(synchronizer.c_str()));
    debugMsg("trex:init", "Injecting global filter for synchronizer horizon");
    filter = dynamic_cast<EUROPA::TiXmlElement *>(xml_cfg->InsertBeforeChild(xml_cfg->FirstChild(),
									     EUROPA::TiXmlElement("FlawFilter")));
  } else {
    debugMsg("trex:init", "Altering planner config for synchronizer");
    debugMsg("trex:init", "Altering global filter for synchronizer horizon");
  }

  filter->SetAttribute("component", TO_STRING_EVAL(TREX_SYNCH_FILT));
  debugMsg("trex:init", "Injecting current state flaws manager with priority of "<<TREX_SYNCH_PRIORITY);

  EUROPA::TiXmlElement synch(TO_STRING_EVAL(TREX_SYNCH_MGR)),
    handler("FlawHandler");
  synch.SetAttribute("defaultPriority", TREX_SYNCH_PRIORITY);
  handler.SetAttribute("component", TO_STRING_EVAL(TREX_SYNCH_HANDLER));
  synch.InsertEndChild(handler);
  xml_cfg->InsertEndChild(synch);

  debugMsg("trex:init", "Configuring europa synchronizer with xml:\n"<<(*xml_cfg));
  m_synchronizer = (new EUROPA::SOLVERS::Solver(plan_db(), *xml_cfg))->getId();

  debugMsg("trex:init", "Adding synchronizer activity listener");
  m_synchronizer->addListener(m_synchListener->getId());
}

EUROPA::ConstrainedVariableId Assembly::get_tick_const() {
  std::ostringstream oss;
  oss<<"__trex_tick_"<<now();
  EUROPA::LabelStr name(oss.str());
  EUROPA::DataTypeId type = m_clock->getDataType();

  if( m_tick_const.isNoId() || m_tick_const->getName()!=name ) {
    UNIQ_PTR<EUROPA::Domain> base(type->baseDomain().copy());
    base->set(now());

    debugMsg("trex:always", "Creating const "<<name.toString());
    m_tick_const = m_cstr_engine->createVariable(type->getName().c_str(),
						 *base, true, true,
						 name.c_str());
    m_tick_const->incRefCount();
  }
  debugMsg("trex:always", "Current tick : "<<m_tick_const->getName().toString()
	   <<'='<<m_tick_const->toString());
  return m_tick_const;
}


void Assembly::notify(details::CurrentState const &state) {
  EUROPA::LabelStr obj_name = state.timeline()->getName();

  if( is_internal(obj_name) ) {
    debugMsg("trex:notify", "Post observation "<<state.timeline()->toString()
             <<'.'<<state.current()->getUnqualifiedPredicateName().toString()<<":\n"
             <<state.current()->toLongString());
    notify(obj_name, state.current());
  } // else if( is_external(obj_name) ) {
    // EUROPA::TokenId cur = state.current();
    // if( cur->isInactive() ) {
  /* Do not do thaT !!! it breaks the plan */
  //   plan_db()->getClient()->activate(cur);
  // }
  // }
}

bool Assembly::commit_externals() {
  bool auto_prop = m_cstr_engine->getAutoPropagation();

  m_cstr_engine->setAutoPropagation(false);

  BOOST_SCOPE_EXIT((&auto_prop)(&m_cstr_engine)) {
    m_cstr_engine->setAutoPropagation(auto_prop);
  } BOOST_SCOPE_EXIT_END;

  for(external_iterator i=begin_external(); end_external()!=i; ++i) {
    if( !(*i)->commit() ) {
      debugMsg("trex:always", "["<<now()
               <<"] failed to integrate state of external timeline "
               <<(*i)->timeline()->getName().toString());
      return false;
    } else {
      EUROPA::TokenId cur = (*i)->current();
      if(cur->isMerged()) {
        EUROPA::TokenId act = cur->getActiveToken();
        cur->cancel();
        plan_db()->getClient()->merge(cur, act);
      }
    }

  }
  return true;
}

bool Assembly::do_synchronize() {
  m_in_synchronization = true;

  BOOST_SCOPE_EXIT((&m_in_synchronization)) {
    m_in_synchronization = false;
  } BOOST_SCOPE_EXIT_END;

  debugMsg("trex:synch", "Execute synchronization with steps="<<m_synchSteps
	   <<" depth="<<m_synchDepth);
  if( !synchronizer()->solve(m_synchSteps, m_synchDepth) ) {
    debugMsg("trex:synch", "Failed to resolve synchronization for tick "<<now());
    if( synchronizer()->getStepCount()>m_synchSteps ) {
      debugMsg("trex:always", "synchronization did exceed its maximum steps ("
               <<m_synchSteps<<')');
    } else if( synchronizer()->getDepth()>m_synchDepth ) {
      debugMsg("trex:always", "synchronization did exceed its maximum depth ("
               <<m_synchDepth<<')');
    }
    return false;
  }
  for(internal_iterator i=begin_internal(); end_internal()!=i; ++i)
    (*i)->commit(); // It was consistent from synchronizer => never fail
  
  // On success clear decisions as they are now assertions
  synchronizer()->clear();
  return true;
}

void Assembly::erase(EUROPA::TokenSet &set, EUROPA::TokenId const &tok) {
  EUROPA::TokenSet::iterator i = set.find(tok);
  if( set.end()!=i ) {
    if( m_iter==i )
      ++m_iter;
    set.erase(i);
  }
}


bool Assembly::relax(bool aggressive) {
  details::is_rejectable rejectable;
  EUROPA::DbClientId cli = plan_db()->getClient();
  std::string relax_name = "RELAX";
  bool auto_prop = m_cstr_engine->getAutoPropagation();

  m_cstr_engine->setAutoPropagation(false);

  BOOST_SCOPE_EXIT((&auto_prop)(&m_cstr_engine)) {
    m_cstr_engine->setAutoPropagation(auto_prop);
  } BOOST_SCOPE_EXIT_END;


  if( aggressive )
    relax_name = "AGGRESSIVE "+relax_name;

  debugMsg("trex:relax", "["<<now()
           <<"] =================== START "<<relax_name<<" =================");
  // Clean up decisions made by solvers
  debugMsg("trex:relax", "Cancelling current decisions");
  // We can just clear them as we just goinfg to do the cleaning ourselve
  synchronizer()->clear();
  planner()->reset();
  m_updated_commit = m_updated_commit || aggressive; // ask for archiving whenever we have a full relax

  // Use m_iter for robust iteration
  m_iter = m_roots.begin();
  debugMsg("trex:relax", "Evaluating "<<m_roots.size()<<" root tokens.");

  while( m_roots.end()!=m_iter ) {
    EUROPA::TokenId tok = *(m_iter++);

    debugMsg("trex:relax", "Evaluating relaxation of "<<tok->toString()
             <<"\n\tPredicate: "<<tok->getPredicateName().toString()
             <<"\n\tObjects: "<<tok->getObject()->toString()
             <<"\n\tstart: "<<tok->start()->lastDomain().toString()
             <<"\n\tend: "<<tok->end()->lastDomain().toString());

    if( tok->end()->baseDomain().getUpperBound()<=now() ) {
      debugMsg("trex:relax", "\t- "<<tok->getKey()
               <<" necessarily ends in the past");
      if( tok->isFact() ) {
        debugMsg("trex:relax", "\t- "<<tok->getKey()<<" is a fact");
        if( aggressive ) {
          debugMsg("trex:relax", "\t- destroying "<<tok->getKey()
                   <<" (aggressive)");
          if( !tok->isInactive() ) {
	    EUROPA::TokenSet slaves = tok->slaves();
	    for(EUROPA::TokenSet::const_iterator s=slaves.begin();
		slaves.end()!=s; ++s) {
	      if( !(*s)->isInactive() )
		cli->cancel(*s);
	    }
            cli->cancel(tok);
	  }
          cli->deleteToken(tok);
        } else if( tok->isMerged() ) {

 	  EUROPA::TokenId active = tok->getActiveToken();
	  active->incRefCount();
          cli->cancel(active);
          if( !aggressive ) {
            debugMsg("trex:relax", "\t- activate fact "<<tok->getKey());
            // cli->activate(tok);
          }
	  active->decRefCount();
        }
      } else {
        bool can_reject = rejectable(tok);

        if( can_reject || aggressive ) {
          if( can_reject ) {
            debugMsg("trex:relax", "\t- destroying the goal "<<tok->getKey());
          } else {
            debugMsg("trex:relax", "\t- destroying past token "<<tok->getKey()
                     <<" (aggressive)");
          }
          if( !tok->isInactive() )
            cli->cancel(tok);
          cli->deleteToken(tok);
        }
      }
    } else {
      if( tok->isFact() ) {
        if( tok->isMerged() ) {
          debugMsg("trex:relax", "\t- give room for fact "<<tok->getKey()
		   <<" (cancel active="<<tok->getActiveToken()->getKey()<<")");
	  EUROPA::TokenId active = tok->getActiveToken();
	  active->incRefCount();
          cli->cancel(active);
          if( !aggressive ) {
            debugMsg("trex:relax", "\t- activate fact "<<tok->getKey());
            // cli->activate(tok);
          }
	  active->decRefCount();
        } /*else if( tok->isActive() ) {
            cli->cancel(tok);
            }*/
      } else if( !tok->isInactive() ) {
        debugMsg("trex:relax", "\t- cancelling non fact "<<tok->getKey());
        cli->cancel(tok);
      }
    }
  }

  debugMsg("trex:relax", "["<<now()<<"] =================== END "
           <<relax_name<<" =================");
  bool ret = constraint_engine()->propagate();
  if( !ret )
    debugMsg("trex:relax", "RELAX FAILURE !!!!");
  return ret;
}

void Assembly::replace(EUROPA::TokenId const &tok) {
  if( !tok->isMerged() )
    throw EuropaException("Attempted to replace a non merged token.");

  EUROPA::ObjectDomain
    const &dom = details::active(tok)->getObject()->lastDomain();
  std::list<EUROPA::ObjectId> objs = dom.makeObjectList();
  // Find the CurrentState if any
  state_iterator pos = m_agent_timelines.find(objs.front()->getKey());
  EUROPA::TokenId active = tok->getActiveToken();

  if( m_agent_timelines.end()!=pos ) {
    (*pos)->replaced(tok);
    tok->cancel();
  } else {
    details::restrict_bases(active, tok);
    tok->cancel();
  }
  if( m_committed.end()==m_committed.find(active) ) {
    m_completed.insert(active);
    m_updated_commit = true;
  }
  // Make dsure that this guy will stay on the current spot
  tok->end()->restrictBaseDomain(active->end()->lastDomain());
  tok->start()->restrictBaseDomain(active->start()->lastDomain());
}

void Assembly::archive(EUROPA::eint date) {
  // return;
  bool auto_prop = m_cstr_engine->getAutoPropagation();
  EUROPA::eint cur = now();

  m_cstr_engine->setAutoPropagation(false);
  m_archiving = true;
  debugMsg("trex:archive",
	   '['<<cur<<"] ============= START archiving ============");

  BOOST_SCOPE_EXIT((&auto_prop)(&m_cstr_engine)(&cur)(&m_archiving)) {
    debugMsg("trex:archive",
             '['<<cur<<"] ============= END archiving ============");
    m_cstr_engine->setAutoPropagation(auto_prop);
    m_archiving = false;
  } BOOST_SCOPE_EXIT_END;
#ifdef TREX_ARCHIVE_Greedy
  EUROPA::DbClientId cli = plan_db()->getClient();
  size_t deleted = 0;
  m_updated_commit = false;
  
  debugMsg("trex:archive", "Checking for completed root tokens");
  m_iter = m_roots.begin();
  while( m_roots.end()!=m_iter ) {
    EUROPA::TokenId tok = *(m_iter++);
    if( !tok->isCommitted() &&
       details::active(tok)->end()->lastDomain().getUpperBound() <= date
       && m_committed.end()!=m_committed.find(tok) ) {
      debugMsg("trex:archive", "Terminating "<<tok->getPredicateName().toString()
               <<'('<<tok->getKey()<<')');
      terminate(tok);
    }
  }

  debugMsg("trex:archive", "Evaluating "<<m_completed.size()
           <<" completed tokens.");
  m_iter = m_completed.begin();
  while( m_completed.end()!=m_iter ) {
    EUROPA::TokenId tok = *(m_iter++);
    if( !tok.isId() ) {
      debugMsg("trex:always", "WARNING: token reference "<<tok<<" is not valid.");
      m_completed.erase(tok);
      continue;
    }       
    std::string type;
    if( is_action(tok) )
      type = "action";
    else
      type = "predicate";
    
    if( details::upperBound(details::active(tok)->end()) > cur ) {
      debugMsg("trex:always", "WARNING: "<<type<<" "
	       <<tok->getPredicateName().toString()<<'('<<tok->getKey()
	       <<") is no longer completed (end=["
	       <<details::lowerBound(details::active(tok)->end())<<", "
	       <<details::upperBound(details::active(tok)->end())<<"])");
      m_completed.erase(tok);
      continue;
    } else {
      EUROPA::TokenId master = tok->master();

      if( master.isNoId() ) {
        if( tok->isInactive() ) {
          debugMsg("trex:archive", "Discard ROOT inactive completed "<<type<<" "
                   <<tok->getPredicateName().toString()<<'('
                   <<tok->getKey()<<')');
          discard(tok);
          tok->discard();
          ++deleted;
        } else if( tok->isMerged() ) {
          EUROPA::TokenId active = tok->getActiveToken();
          if( m_committed.find(active)!=m_committed.end() ) {
            debugMsg("trex:archive", "Discard ROOT merged completed "<<type<<" "
                     <<tok->getPredicateName().toString()<<'('
                     <<tok->getKey()<<')');
            replace(tok);
            discard(tok);
            tok->discard();
            ++deleted;
          } else if( m_completed.find(active)==m_completed.end() ) {
            details::restrict_base(tok, tok->start(),
                                   tok->start()->lastDomain());
            details::restrict_base(tok, tok->end(), tok->end()->lastDomain());
            terminate(active);
          }
        } else if( tok->isActive() ) {
          debugMsg("trex:archive", "Commit ROOT active completed "<<type<<" "
                   <<tok->getPredicateName().toString()<<'('
                   <<tok->getKey()<<')');
          m_committed.insert(tok);
          m_completed.erase(tok);
          m_updated_commit = true;
        }
      } else if( m_committed.end()!=m_committed.find(master) ) {
        if( tok->isMerged() ) {
          EUROPA::TokenId active = tok->getActiveToken();
          if( m_committed.find(active)!=m_committed.end() ) {
            debugMsg("trex:archive", "Cancel merged completed "<<type<<" "
                     <<tok->getPredicateName().toString()<<'('
                     <<tok->getKey()<<')');
            replace(tok);
          } else if( m_completed.find(active)==m_completed.end() ) {
            debugMsg("trex:archive", "Marking active ("<<master->getKey()
                     <<") counterpart of "<<type<<" "
                     <<tok->getPredicateName().toString()<<'('
                     <<tok->getKey()<<") as completed");
            terminate(active);
          }
        } else if( tok->isActive() ) {
          debugMsg("trex:archive", "Making active "<<type<<" "
                   <<tok->getPredicateName().toString()<<'('
                   <<tok->getKey()<<") committed");
          m_committed.insert(tok);
          m_completed.erase(tok);
          m_updated_commit = true;
        } else {
          debugMsg("trex:archive", type<<" "<<tok->getPredicateName().toString()
                   <<'('<<tok->getKey()<<") is completed but neither active nor merged");
        }
      } else if( details::upperBound(master->end())<=cur ) {
        debugMsg("trex:archive", type<<" "<<tok->getPredicateName().toString()
                 <<'('<<tok->getKey()<<") marked as completed as its end="
                 <<master->end()->lastDomain()<<"<="<<cur);
        terminate(master);
      }
    }
  }

  debugMsg("trex:archive", "Evaluate "<<m_committed.size()
           <<" committed tokens");
  m_iter = m_committed.begin();
  while( m_committed.end()!=m_iter ) {
    EUROPA::TokenId tok = *(m_iter++);
    if( tok->isActive() ) {
      bool can_delete = true;
      EUROPA::TokenSet tmp = tok->getMergedTokens();
      // Check that all masters are committed
      for(EUROPA::TokenSet::const_iterator i=tmp.begin();
          tmp.end()!=i; ++i) {
        EUROPA::TokenId master = (*i)->master();
        if( master.isId() ) {
          if( m_committed.find(master)==m_committed.end() ) {
            if( can_delete ) {
              debugMsg("trex:archive", "Cannot delete "
                       <<tok->getPredicateName().toString()<<'('
                       <<tok->getKey()
                       <<"): one of its master is not yet committed:\n\t-"
                       <<(is_action(master)?"action":"predicate")
                       <<" "<<master->getPredicateName().toString()<<'('<<master->getKey()<<")");
              can_delete = false;
            }
            if( details::upperBound(master->end())<=date ) {
              debugMsg("trex:archive", "Adding "<<tok->getPredicateName().toString()
                       <<'('<<tok->getKey()<<")'s master "<<(is_action(master)?"action":"predicate")
                       <<" "<<master->getPredicateName().toString()<<'('<<master->getKey()<<") to completed list");
              terminate(master);
            }
          }
        }
      }
      // Check its slaves
      tmp = tok->slaves();

      for(EUROPA::TokenSet::const_iterator i=tmp.begin();
          tmp.end()!=i; ++i) {
        EUROPA::TokenId i_active = details::active(*i);
        
        if( details::upperBound(i_active->end())<=date ) {
          if( !(*i)->isInactive() ) {
            EUROPA::TokenSet::const_iterator j = m_committed.find(i_active);
            
            if(m_committed.end()==j ) {
              if( can_delete ) {
                debugMsg("trex:archive", "Cannot delete "<<(is_action(tok)?"action":"predicate")<<" "
                         <<tok->getPredicateName().toString()<<'('
                         <<tok->getKey()
                         <<"): one of its slaves is part of the plan:\n\t- "
                         <<(is_action(*i)?"action ":"predicate ")<<(*i)->getPredicateName().toString()
                         <<'('<<(*i)->getKey()<<") |"<<details::active(*i)->start()->lastDomain()<<", "
                         <<details::active(*i)->end()->lastDomain()<<"|.");
                can_delete = false;
              }
              if( m_completed.end()!=m_completed.find(*i) ) {
                debugMsg("trex:archive", "Adding slave "<<(is_action(*i)?"action ":"predicate ")
                         <<(*i)->getPredicateName().toString()
                         <<'('<<(*i)->getKey()<<") to completed tokens.");
                terminate(*i);
              }
            } else {
              if( (*i)->isMerged() ) {
                debugMsg("trex:archive", "Cancel merged completed slave "<<(is_action(*i)?"action ":"predicate ")
                         <<(*i)->getPredicateName().toString()<<'('
                         <<(*i)->getKey()<<')');
                replace(*i);
              } else {
                // Tricky situation where I need to try to cancl thistoken without killing (*i)
              }
            }
          }
      
        } else if( details::upperBound(i_active->start())<date ) {
          EUROPA::ObjectDomain const &dom = (*i)->getObject()->lastDomain();
          EUROPA::ObjectId obj = dom.makeObjectList().front();
          state_iterator pos = m_agent_timelines.find(obj->getKey());
          if( m_agent_timelines.end()==pos ||
              !((*pos)->external() && 0==look_ahead(obj)) ) {
            if( can_delete ) {
              debugMsg("trex:archive", "Cannot delete "<<(is_action(tok)?"action":"predicate")<<" "
                       <<tok->getPredicateName().toString()<<'('
                       <<tok->getKey()
                       <<"): one of its slaves is not finished yet:\n\t- "
                       <<(is_action(*i)?"action ":"predicate ")<<(*i)->getPredicateName().toString()
                       <<'('<<(*i)->getKey()<<')');
              can_delete = false;
            }
          }
        } else {
          if( can_delete ) {
            debugMsg("trex:archive", "Cannot delete "<<(is_action(tok)?"action":"predicate")<<" "
                     <<tok->getPredicateName().toString()<<'('
                     <<tok->getKey()
                     <<"): one of its slaves is not yet started ("<<date<<"):\n\t- "
                     <<(is_action(*i)?"action ":"predicate ")<<(*i)->getPredicateName().toString()
                     <<'('<<(*i)->getKey()<<':'<<i_active->getKey()<<")["<<(*i)->getState()->lastDomain().toString()<<"]"
 		     <<"\n\t    start=="<<i_active->start()->toString()<<"<="<<details::upperBound(i_active->start())
 		     <<"\n\t    end=="<<i_active->end()->toString()<<"<="<<details::upperBound(i_active->end()));
            can_delete = false;
          }
        }
      }
      if( can_delete ) {
        debugMsg("trex:archive", "Destroy "<<(is_action(tok)?"action":"predicate")<<" "
                 <<tok->getPredicateName().toString()<<'('<<tok->getKey()
                 <<") as all its slaves are now inactives and in the past.");
        discard(tok);
	EUROPA::IntervalIntDomain st = tok->start()->lastDomain(), en = tok->end()->lastDomain();
	
	details::restrict_bases(tok);
	if( tok->isActive() ) {
	  // Need to propagate the values to all the merged tokens
	  // NOTE: this is not very acceptable as I may lose this information
	  //       should the merged tokens been removed ... ideally I should 
	  //       activate another one instead and/or make sured that the one 
	  //       to be activated is marked as potential deletion
	  //       Still it worked fine in the case of lsts PLanner to forget 
	  //       the past and something like this is obviously need to ensure 
	  //       proper archiving
	  EUROPA::TokenSet merged = tok->getMergedTokens();
	  cli->cancel(tok);
	  for(EUROPA::TokenSet::iterator i=merged.begin(); merged.end()!=i; ++i)
	    details::restrict_bases(*i, tok);
	} else
	  cli->cancel(tok);
        if( tok->master().isNoId() )
          tok->discard();
        ++deleted;
      }
    } else {
      debugMsg("trex:archive", (is_action(tok)?"action":"predicate")<<" "<<tok->getPredicateName().toString()
               <<'('<<tok->getKey()<<") is not active.\n\tSet it complete.");
      m_committed.erase(tok);
      terminate(tok);
    }
  }

  // Needf
  debugMsg("trex:archive", "Archived "<<deleted<<" tokens [updated="<<m_updated_commit<<"].");
  constraint_engine()->propagate();

  // plan_db()->archive(now()-1);

#else // TREX_ARCHIVE_EuropaDefault

  // Just rely on the europa archival : safe but inefficient
  plan_db()->archive(date-1);

#endif
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
  
  //std::cerr<<"Created "<<tok->getPredicateName().toString()<<'('<<tok->getKey()<<')'<<std::endl;
  return tok;
}

EUROPA::TokenId Assembly::new_obs(EUROPA::ObjectId const &obj,
                                  std::string &pred,
                                  bool &undefined) {
  if( !external(obj) )
    throw EuropaException(obj->toString()+"is not an External timeline");
  state_iterator handler = m_agent_timelines.find(obj->getKey());

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
  if( tok.isId() ) {
    // Very aggressive way to handle this
    debugMsg("trex:recall", "Goal "<<tok->toString()<<" recalled.");

    if( !tok->isInactive() ) {
      debugMsg("trex:recall", "Cancelling "<<tok->toString());
      plan_db()->getClient()->cancel(tok);
    }
    debugMsg("trex:recall", "Destroying "<<tok->toString());
    plan_db()->getClient()->deleteToken(tok);
  } else
    debugMsg("trex:always",
             "["<<now()<<"] Attempted to recall an invalid token.");
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

bool Assembly::is_action(EUROPA::TokenId const &tok) const {
#ifdef EUROPA_HAVE_EFFECT
  // std::cerr<<"EVAL is_action("<<tok->getPredicateName().toString()<<"("<<tok->getKey()<<"))"<<std::endl;
  return tok->hasAttributes(EUROPA::PSTokenType::ACTION);
#else
  return false;
#endif
}

bool Assembly::is_predicate(EUROPA::TokenId const &tok) const {
#ifdef EUROPA_HAVE_EFFECT
  return tok->hasAttributes(EUROPA::PSTokenType::PREDICATE);
#else
  return true;
#endif
}


bool Assembly::is_condition(EUROPA::TokenId const &tok) const {
#ifdef EUROPA_HAVE_EFFECT
  return tok->hasAttributes(EUROPA::PSTokenType::CONDITION);
#else
  return false;
#endif
}

bool Assembly::is_effect(EUROPA::TokenId const &tok) const {
#ifdef EUROPA_HAVE_EFFECT
  return tok->hasAttributes(EUROPA::PSTokenType::EFFECT);
#else
  return false;
#endif
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

size_t Assembly::look_ahead(EUROPA::ObjectId const &obj) {
  if( !is_agent_timeline(obj) )
    return 0;
  return look_ahead(obj->getName());
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

std::ostream &Assembly::print_domain(std::ostream &out,
                            EUROPA::ConstrainedVariableId const &var) const {
  EUROPA::Domain const &dom = var->lastDomain();

  if( dom.isSingleton() ) {
    EUROPA::edouble val = dom.getSingletonValue();

    if( m_schema->isObjectType(dom.getTypeName()) )
      return out<<dom.toString();
    else {
      EUROPA::DataTypeId type = var->getDataType();
      return out<<type->toString(val);
    }
  } else if( dom.isInterval() ) {
    return out<<dom;
  } else {
    return out<<dom.toString();
  }
}


void Assembly::print_plan(std::ostream &out, bool expanded) const {
  EUROPA::TokenSet const tokens = plan_db()->getTokens();
  is_not_merged filter(false);
  std::set<EUROPA::eint> instants;

  out<<"digraph plan_"<<now()<<" {\n"
     <<"  node[shape=\"box\"];\n\n";
  if( !expanded )
    out<<"  graph[rankdir=\"LR\"];\n";
  boost::filter_iterator<is_not_merged, EUROPA::TokenSet::const_iterator>
    it(filter, tokens.begin(), tokens.end()),
    endi(filter, tokens.end(), tokens.end());
  // Iterate through plan tokens
  for( ; endi!=it; ++it) {
    std::string name;
    EUROPA::ObjectVarId obj = (*it)->getObject();
    if( obj->getLastDomain().isSingleton() ) {
      std::list<EUROPA::ObjectId> objs = obj->getLastDomain().makeObjectList();
      std::ostringstream oss;
      oss<<objs.front()->getName().toString()<<'.'<<(*it)->getUnqualifiedPredicateName().toString();
      name = oss.str();
    } else 
      name = (*it)->getPredicateName().toString();
    

    EUROPA::eint key = (*it)->getKey();
    // display the token as a node
    out<<"  t"<<key<<"[label=\""<<name
       <<'('<<key<<") {\\n";
    if( (*it)->isIncomplete() )
      out<<"incomplete\\n";
    out<<"nref="<<(*it)->refCount()<<"\\n";
    if( !(*it)->isInactive() )
      print_domain(out<<"  STATE: "<<std::flush, (*it)->getState())
        <<"\\n"<<std::flush;
    else
      out<<"  STATE: INACTIVE\\n"<<std::flush;
#ifdef EUROPA_HAVE_EFFECT
    out<<"type: ";
    if( is_action(*it) )
      out<<"ACTION";
    else if( is_predicate(*it) )
      out<<"PREDICATE";
    else
      out<<"???";
    out<<"\\n"<<std::flush;
#endif // EUROPA_HAVE_EFFECT
    print_domain(out<<"  start="<<std::flush, (*it)->start());
    print_domain(out<<"\\n  duration="<<std::flush, (*it)->duration());
    print_domain(out<<"\\n  end="<<std::flush, (*it)->end())<<"\\n"<<std::flush;

    std::vector<EUROPA::ConstrainedVariableId> const &attrs = (*it)->parameters();

    for(std::vector<EUROPA::ConstrainedVariableId>::const_iterator a=attrs.begin();
        attrs.end()!=a; ++a)
      print_domain(out<<"  "<<(*a)->getName().toString()<<'='<<std::flush, *a)<<"\\n";

    if( (*it)->isActive() ) {
      EUROPA::TokenSet const &merged = (*it)->getMergedTokens();
      if( !merged.empty() ) {
        out<<"merged={";
        EUROPA::TokenSet::const_iterator m = merged.begin();
        out<<(*m)->getKey()<<'['<<(*m)->refCount()<<']';
        for(++m; merged.end()!=m; ++m)
          out<<", "<<(*m)->getKey()<<'['<<(*m)->refCount()<<']';
        out<<"}\\n";
      }
    }
    out<<"}\"";
    if( ignored(*it) )
      out<<" color=grey"; // ignored tokens are greyed
    else if( filter.is_fact(*it) )
      out<<" color=red"; // fact tokens are red
    else if(m_goals.find(*it)!=m_goals.end())
      out<<" color=blue";
    if( (*it)->isCommitted() ||
        m_committed.find(*it)!=m_committed.end() )
      out<<" fontcolor=red";
    std::ostringstream styles;
    bool comma = false;

    if( filter.is_goal(*it) ) {
      styles<<"rounded"; // goal have rounded corner
      comma=true;
    }
    if( m_completed.end()!=m_completed.find(*it) ) {
      if( comma )
        styles.put(',');
      else
        comma = true;
      styles<<"dashed";
    }
#ifdef EUROPA_HAVE_EFFECT
    if( is_action(*it) ) {
      if( comma )
        styles.put(',');
      else
        comma = true;
      styles<<"filled"; // actions are filled
    }
#endif // EUROPA_HAVE_EFFECT
    if( comma )
      out<<" style=\""<<styles.str()<<"\" "; // display style modifiers
    out<<"];"<<std::endl;
    if( (*it)->isMerged() ) {
      EUROPA::eint active = (*it)->getActiveToken()->getKey();
      // connect the merged token to its active counterpart
      out<<"  t"<<key<<"->t"<<active<<"[color=grey];\n";
    }
    if( expanded ) {
      EUROPA::TokenSet toks;
      toks.insert(*it);
      filter.merged(*it, toks);
      // display the relation to the master token(s)
      for(EUROPA::TokenSet::const_iterator t=toks.begin(); toks.end()!=t; ++t) {
        EUROPA::TokenId master = (*t)->master();

        if( master.isId() ) {
          out<<"  t"<<master->getKey()<<"->t"<<key
             <<"[label=\""<<(*t)->getRelation().toString();
#ifdef EUROPA_HAVE_EFFECT
          if( is_effect(*t) )
            out<<"\\n(effect)";
          if( is_condition(*t) )
            out<<"\\n(condition)";
#endif // EUROPA_HAVE_EFFECT
          out<<"\"];\n";
          // if( (*it)!=(*t) )
          //   out<<" color=grey";
        }
      }
    } else {
      EUROPA::eint lb, ub;
      lb = (*it)->start()->lastDomain().getLowerBound();
      // ub = (*it)->start()->lastDomain().getUpperBound();

      if( lb>std::numeric_limits<EUROPA::eint>::minus_infinity() ) {
        if( instants.insert(lb).second )
          out<<"  \"i"<<lb<<"\"[shape=point, label=\""<<lb<<"\"];\n";
        out<<"  \"i"<<lb<<"\"->t"<<(*it)->getKey()<<"[color=grey style=dashed weight=1000];\n";
      }
      // if( ub<std::numeric_limits<EUROPA::eint>::infinity() ) {
      //   if( instants.insert(ub).second )
      // 	out<<"  i"<<ub<<"[shape=point, label=\""<<ub<<"\"];\n";
      //   out<<"  t"<<(*it)->getKey()<<"->i"<<ub"[weight=10.0];\n";
      // }

      // lb = (*it)->start()->lastDomain().getLowerBound();
      ub = (*it)->end()->lastDomain().getUpperBound();

      // if( lb>std::numeric_limits<EUROPA::eint>::minus_infinity() ) {
      //   if( instants.insert(lb).second )
      // 	out<<"  i"<<lb<<"[shape=point, label=\""<<lb<<"\"];\n";
      //   out<<"  i"<<lb<<"->t"<<(*it)->getKey()<<"[weight=10.0];\n";
      // }
      if( ub<std::numeric_limits<EUROPA::eint>::infinity() ) {
        if( instants.insert(ub).second )
          out<<"  \"i"<<ub<<"\"[shape=point, label=\""<<ub<<"\"];\n";
        out<<"  t"<<(*it)->getKey()<<"->\"i"<<ub<<"\"[color=grey style=dashed weight=1000];\n";
      }
    }
  }
  if( !instants.empty() ) {
    std::set<EUROPA::eint>::const_iterator i=instants.begin();
    EUROPA::eint pred = *(i++);
    EUROPA::eint max = 100+(*instants.rbegin())-pred;
    out<<"  subgraph instants_cluster {\n"
       <<"   node[shape=point];\n"
       <<"   edge[color=none];\n"
       <<"   \"i"<<pred<<"\"[label=\""<<pred<<"\"];\n";

    for(;instants.end()!=i; ++i) {
      out<<"   \"i"<<pred<<"\"->\"i"<<(*i)<<"\"[weight=\""
         <<(max-((*i)-pred))<<"\"];\n";
      pred = *i;
      out<<"   \"i"<<pred<<"\"[label=\""<<pred<<"\"];\n";
    }
    out<<"  }\n";
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

void Assembly::backtracking(EUROPA::SOLVERS::DecisionPointId &dp) {
  debugMsg("trex:always", "["<<now()<<"] Last decision : "<<m_synchronizer->getLastExecutedDecision());
}

void Assembly::print_context(std::ostream &out, EUROPA::ConstrainedVariableId const &v) const {
  // EUROPA::TokenId tok = details::parent_token(v);
  details::var_print(out<<"Local context for ", v)<<":\n";
  out<<"  - base domain: "<<v->baseDomain().toString()
     <<"\n  - last domain: "<<v->lastDomain().toString()
     <<"\n  - europa violation explanation: "<<v->getViolationExpl()<<'\n';
  EUROPA::ConstraintSet constraints;
  v->constraints(constraints);
  if( !constraints.empty() ) {
    out<<"  - related constraints:\n";
    for(EUROPA::ConstraintSet::const_iterator c=constraints.begin(); constraints.end()!=c; ++c) {
      EUROPA::ConstraintId cstr = *c;
      std::vector<EUROPA::ConstrainedVariableId> const &scope = cstr->getScope();
      size_t i, end_i = scope.size();

      out<<"\t+ "<<cstr->getName().toString()<<'(';
      for(i=0; i<end_i; ++i) {
        if( i>0 )
          out<<", ";
        details::var_print(out, scope[i]);
        if( scope[i]!=v )
          out<<"="<<scope[i]->lastDomain().toString();
      }
      out<<")\n";
    }
  }
}

/*
 * class TREX::europa::Assembly::ce_listener
 */

Assembly::ce_listener::ce_listener(Assembly &owner)
  :EUROPA::ConstraintEngineListener(owner.m_cstr_engine), m_owner(owner) {}

void Assembly::ce_listener::notifyPropagationPreempted() {
  if( m_owner.m_in_synchronization ) {
    debugMsg("trex:always", "["<<m_owner.now()<<"] search preempted during synchronization");

    if( !m_empty_vars.empty() ) {
      debugMsg("trex:always", "======================================================================");
      debugMsg("trex:always", "["<<m_owner.now()<<"] "<<m_empty_vars.size()<<" variables are empty:");
      for(EUROPA::ConstrainedVariableSet::const_iterator v=m_empty_vars.begin(); m_empty_vars.end()!=v; ++v) {
        std::ostringstream oss;
        m_owner.print_context(oss, *v);
        debugMsg("trex:always", oss.str()<<"\n");
      }
      debugMsg("trex:always", "======================================================================");
    }
  }
}

/*
 * class TREX::europa::Assembly::synchronization_listener
 */

void Assembly::synchronization_listener::notifyCreated(EUROPA::SOLVERS::DecisionPointId& dp) {
  debugMsg("trex:synch:search", "New decision point ["<<dp->getKey()<<"]");
  m_progress = true;
}

void Assembly::synchronization_listener::notifyDeleted(EUROPA::SOLVERS::DecisionPointId& dp) {
  debugMsg("trex:synch:search", "Delete decision point ["<<dp->getKey()<<"](backtrack)");
}

void Assembly::synchronization_listener::notifyStepSucceeded(EUROPA::SOLVERS::DecisionPointId& dp) {
  debugMsg("trex:synch:search", "Executed decision point ["<<dp->getKey()<<"]: "<<dp->toString());
  m_progress = true;
}

void Assembly::synchronization_listener::notifyStepFailed(EUROPA::SOLVERS::DecisionPointId &dp) {
  // This callback is not active yet but is the one I want
  // ... should be available on europa 2.7 (or any version number after 2.6)
  debugMsg("trex:synch:search", "Failed decision point ["<<dp->getKey()<<"]: "<<dp->toString());
}

void Assembly::synchronization_listener::notifyUndone(EUROPA::SOLVERS::DecisionPointId &dp) {
  debugMsg("trex:synch:search", "Undid decision point ["<<dp->getKey()<<"]");
}

void Assembly::synchronization_listener::notifyRetractSucceeded(EUROPA::SOLVERS::DecisionPointId& dp) {
  m_progress = true;
  debugMsg("trex:always", "["<<m_owner.now()<<"] Backtrack completed (depth="<<m_owner.synchronizer()->getDepth()<<")");
}

void Assembly::synchronization_listener::notifyRetractNotDone(EUROPA::SOLVERS::DecisionPointId& dp) {
  if( m_progress ) {
    m_progress = false;
    debugMsg("trex:always", "["<<m_owner.now()<<"] start to backtrack (depth="<<m_owner.synchronizer()->getDepth()<<")");
    m_owner.backtracking(dp);
  }
}

/*
 * class TREX::europa::Assembly::listener_proxy
 */

void Assembly::listener_proxy::notifyAdded(EUROPA::TokenId const &token) {
    // Adds goal when added to the plan
    thread_duration duration;
    thread_clock::time_point start = thread_clock::now();
    if(m_owner.is_goal(token)) {
      m_owner.m_goals.insert(token);
    }
    duration = thread_clock::now()-start;
    if(m_owner.time_values.find(m_owner.now())==m_owner.time_values.end())
        m_owner.time_values[m_owner.now()]=duration.count();
    else
        m_owner.time_values[m_owner.now()]+=duration.count();
    //std::cout<<"Added: "<<m_owner.now()<<": "<<m_owner.time_values[m_owner.now()]<<std::endl;

    EUROPA::TokenId master = token->master();
    if( master.isNoId() ) {
      m_owner.m_roots.insert(token);
      m_owner.m_updated_commit = true;
    // token->incRefCount();
    }
}

void Assembly::listener_proxy::notifyRemoved(EUROPA::TokenId const &token) {
  // Removes goal when removed from plan
  thread_duration duration;
  thread_clock::time_point start = thread_clock::now();
  m_owner.m_goals.erase(token);
  duration = thread_clock::now()-start;
    if(m_owner.time_values.find(m_owner.now())==m_owner.time_values.end())
        m_owner.time_values[m_owner.now()]=duration.count();
    else
        m_owner.time_values[m_owner.now()]+=duration.count();
  //std::cout<<"Removed: "<<m_owner.now()<<": "<<m_owner.time_values[m_owner.now()]<<std::endl;

  m_owner.erase(m_owner.m_roots, token);
  m_owner.erase(m_owner.m_completed, token);
  m_owner.erase(m_owner.m_committed, token);

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

void Assembly::listener_proxy::notifyActivated(EUROPA::TokenId const &token)
{
    thread_duration duration;
    thread_clock::time_point start = thread_clock::now();

    if(m_owner.is_goal(token))
    {
        EUROPA::TokenSet tokens;
        tokens.insert(token);
        // Searchs for and adds all subgoals
        m_owner.subgoalSearch(tokens);
    } else if(m_owner.conditionOfGoal(token)) {
        EUROPA::TokenSet tokens;
        tokens.insert(token);
        m_owner.subgoalSearch(tokens);
    }

    duration = thread_clock::now()-start;
    if(m_owner.time_values.find(m_owner.now())==m_owner.time_values.end())
        m_owner.time_values[m_owner.now()]=duration.count();
    else
        m_owner.time_values[m_owner.now()]+=duration.count();
    //std::cout<<"Activated: "<<m_owner.now()<<": "<<m_owner.time_values[m_owner.now()]<<std::endl;
}

void Assembly::listener_proxy::notifyDeactivated(EUROPA::TokenId const &token) {
  // Checks and erases the token if it was considered a goal
  thread_duration duration;
  thread_clock::time_point start = thread_clock::now();
  if(!m_owner.is_goal(token) && !m_owner.conditionOfGoal(token) && m_owner.is_subgoal(token))
  {
    EUROPA::TokenSet temp;
    temp.insert(token);
    m_owner.removeSubgoals(temp);
  }

  duration = thread_clock::now()-start;
    if(m_owner.time_values.find(m_owner.now())==m_owner.time_values.end())
        m_owner.time_values[m_owner.now()]=duration.count();
    else
        m_owner.time_values[m_owner.now()]+=duration.count();
  //std::cout<<"Deactived: "<<m_owner.now()<<": "<<m_owner.time_values[m_owner.now()]<<std::endl;

  if( m_owner.is_agent_timeline(token) ) {
    debugMsg("trex:token", "cancel "<<token->getPredicateName().toString()
             <<'('<<token->getKey()<<')');
    if( !m_owner.m_archiving )
      m_owner.cancel(token); // do not recall during archiving !!!!
  }
}

void Assembly::listener_proxy::notifyMerged(EUROPA::TokenId const &token) {
    thread_duration duration;
    thread_clock::time_point start = thread_clock::now();
    m_owner.m_masters[token] = token->getActiveToken();
    if(m_owner.is_goal(token) || m_owner.is_subgoal(token))
    {
        EUROPA::TokenSet tokens = token->getActiveToken()->getMergedTokens();
        tokens.insert(token->getActiveToken());
        // Searchs for and adds all subgoals
        m_owner.subgoalSearch(tokens);
    }
    // Check to find if the active token is in a goal, then it search for effect and conditions
    else if(m_owner.is_subgoal(token->getActiveToken()))
    {
        EUROPA::TokenSet tokens;
        tokens.insert(token);
        // Searchs for and adds all subgoals
        m_owner.subgoalSearch(tokens);
    }

    duration = thread_clock::now()-start;
    if(m_owner.time_values.find(m_owner.now())==m_owner.time_values.end())
        m_owner.time_values[m_owner.now()]=duration.count();
    else
        m_owner.time_values[m_owner.now()]+=duration.count();
    //std::cout<<"Merged: "<<m_owner.now()<<": "<<m_owner.time_values[m_owner.now()]<<std::endl;
}

void Assembly::listener_proxy::notifySplit(EUROPA::TokenId const &token) {
  // Checks and erases the token if it was considered a goal
  thread_duration duration;
  thread_clock::time_point start = thread_clock::now();
  if(m_owner.is_subgoal(token) && m_owner.conditionOfGoal(token))
  {
      EUROPA::TokenSet temp = m_owner.m_masters[token]->getMergedTokens();
      temp.insert(m_owner.m_masters[token]);
      EUROPA::TokenSet::iterator it;
      bool remove = true;
      for(it=temp.begin();  it!=temp.end(); ++it)
      {
          if(m_owner.is_goal(*it) || m_owner.conditionOfGoal(*it))
          {
              remove = false;
          }
      }
      if(remove)
      {
          m_owner.removeSubgoals(temp);
      }
  } else {
     EUROPA::TokenSet temp;
     temp.insert(token);
     m_owner.removeSubgoals(temp);
  }
  m_owner.m_masters.erase(token);

  duration = thread_clock::now()-start;
    if(m_owner.time_values.find(m_owner.now())==m_owner.time_values.end())
        m_owner.time_values[m_owner.now()]=duration.count();
    else
        m_owner.time_values[m_owner.now()]+=duration.count();
  //std::cout<<"Split: "<<m_owner.now()<<": "<<m_owner.time_values[m_owner.now()]<<std::endl;

  // EUROPA::TokenId master = token->master();
  // if( master.isId() )
  //   token->decRefCount();
}

void Assembly::listener_proxy::notifyRejected(EUROPA::TokenId const &token) {
  debugMsg("trex:always", "["<<m_owner.now()<<"] Token "
           <<token->getPredicateName().toString()<<'('
           <<token->getKey()<<") is rejected.");
  m_owner.rejected(token);
}

void Assembly::listener_proxy::notifyReinstated(EUROPA::TokenId const &token) {
  debugMsg("trex:always", "["<<m_owner.now()<<"] Token "
           <<token->getPredicateName().toString()<<'('
           <<token->getKey()<<") is no longer rejected.");
}

void Assembly::listener_proxy::notifyCommitted(EUROPA::TokenId const &token) {
  m_owner.erase(m_owner.m_completed, token);
  m_owner.m_committed.insert(token);
}

void Assembly::listener_proxy::notifyTerminated(EUROPA::TokenId const &token) {

}

/**
    Listener_proxy helper functions
*/
bool Assembly::actionEffect(const EUROPA::TokenId& token) {
  EUROPA::TokenId master = token->master();
  if(master.isId()) {
    if(is_action(master) && is_effect(token))
      return true;
  }
  return false;
}

bool Assembly::conditionOfGoal(const EUROPA::TokenId& token)
{
    if(token->master().isId())
        if(is_action(token->master()) && is_condition(token))
        {
            EUROPA::TokenSet slaves = token->master()->slaves();
            EUROPA::TokenSet::iterator it = slaves.begin(), end = slaves.end();
            for(; it!=end; it++)
            {
                if(is_effect(*it) && is_subgoal(*it))
                {
                    return true;
                }
            }
        }
    return false;
}

EUROPA::TokenSet Assembly::conditions(const EUROPA::TokenId& token)
{
    EUROPA::TokenSet slaves = token->slaves(), conditions;
    for(EUROPA::TokenSet::iterator it = slaves.begin(), end = slaves.end();
        it!=end; it++)
    {
        if(is_condition((*it)))
            conditions.insert((*it));
    }
    EUROPA::TokenSet merged;
    for(EUROPA::TokenSet::iterator it = conditions.begin(), end = conditions.end();
        it!=end; it++)
    {
        if((*it)->isActive())
        {
            EUROPA::TokenSet temp = (*it)->getMergedTokens();
            merged.insert(temp.begin(), temp.end());
        } else {
            EUROPA::TokenId active = (*it)->getActiveToken();
            if(active.isId())
            {
                EUROPA::TokenSet temp = active->getMergedTokens();
                merged.insert(temp.begin(), temp.end());
                merged.insert(active);
            }
        }
    }
    conditions.insert(merged.begin(),merged.end());
    return conditions;
}

bool Assembly::is_subgoal(const EUROPA::TokenId& token)
{
    return m_goals.find(token)!=m_goals.end();
}

void Assembly::subgoalSearch(EUROPA::TokenSet& tokens)
{
    EUROPA::TokenSet::iterator it, condIt;
    m_goals.insert(tokens.begin(), tokens.end());
    while(!tokens.empty())
    {
        it = tokens.begin();
        if(actionEffect((*it)))
        {
            EUROPA::TokenSet conds = conditions((*it)->master());
            for(condIt=conds.begin(); condIt!=conds.end(); ++condIt)
            {
                if(m_goals.insert(*condIt).second==true)
                    tokens.insert(*condIt);
            }
        }
        tokens.erase(it);
    }
}

void Assembly::removeSubgoals(EUROPA::TokenSet tokens)
{
    EUROPA::TokenSet::iterator it;
    while(!tokens.empty())
    {
        it = tokens.begin();
        m_goals.erase(*it);
        if(actionEffect((*it)))
        {
            EUROPA::TokenSet conds = conditions((*it)->master());
            for(EUROPA::TokenSet::iterator cond=conds.begin(); cond!= conds.end(); ++cond)
            {
                if(m_goals.erase(*cond)>=1)
                    tokens.insert(*cond);
            }
        }
        tokens.erase(it);
    }
}

/*
  # ifdef EUROPA_HAVE_EFFECT
  #  warning "Greedy plan archiving is not efficient with action tokens."
  # endif // EUROPA_HAVE_EFFECT
  EUROPA::DbClientId cli = plan_db()->getClient();
  size_t deleted = 0;

  debugMsg("trex:archive",
  '['<<now()<<"] ============= START archiving ============");

  debugMsg("trex:archive", "Evaluating "<<m_completed.size()<<" completed tokens.");
  m_iter = m_completed.begin();
  while( m_completed.end()!=m_iter ) {
  EUROPA::TokenId tok = *(m_iter++);

  if( tok->canBeCommitted() ) {
  // details::restrict_bases(tok);
  // constraint_engine()->propagate();
  EUROPA::TokenId master = tok->master();

  if( master.isId() ) {
  if( master->isCommitted() ) {
  debugMsg("trex:archive",
  "Committing "<<tok->getPredicateName().toString()
  <<'('<<tok->getKey()<<") as its master is committed.");
  tok->commit();
  } else if( master->isFact() &&
  master->start()->lastDomain().getUpperBound()<now() ) {
  debugMsg("trex:archive",
  "Committing "<<tok->getPredicateName().toString()
  <<'('<<tok->getKey()<<") as its master is a past fact.");
  tok->commit();
  } else {
  debugMsg("trex:archive", "Cannot commit "
  <<tok->getPredicateName().toString()
  <<'('<<tok->getKey()<<") as its master is neither a fact "
  <<"or committed.");
  }
  } else {
  debugMsg("trex:archive",
  "Committing "<<tok->getPredicateName().toString()
  <<'('<<tok->getKey()<<") as it is a root token.");
  tok->commit();
  }
  } else if( tok->isInactive() ) {
  EUROPA::TokenId master = tok->master();
  if( master.isNoId() ) {
  debugMsg("trex:archive", "Discarding inactive orphan token "
  <<tok->getPredicateName().toString()
  <<'('<<tok->getKey()<<").");
  discard(tok);
  tok->discard();
  // constraint_engine()->propagate();
  debugMsg("trex:archive", EUROPA::PlanDatabaseWriter::toString(plan_db()));

  } else if( master->end()->lastDomain().getUpperBound()<=now() ) {
  if( master->isCommitted() ) {
  debugMsg("trex:archive", "Ignoring inactive justified token "
  <<tok->getPredicateName().toString()
  <<'('<<tok->getKey()<<").");
  m_completed.erase(tok);
  } else
  m_completed.insert(master);
  }
  } else {
  // token is merged
  EUROPA::TokenId active = tok->getActiveToken();
  EUROPA::TokenId master = tok->master();

  if( master.isId() ) {
  if( master->end()->lastDomain().getUpperBound()<=now() ) {
  if( !master->isCommitted() )
  m_completed.insert(master);
  }
  }
  if( master.isNoId() || master->isCommitted() ) {
  debugMsg("trex:archive", "Collapsing "
  <<tok->getPredicateName().toString()<<'('<<tok->getKey()
  <<") with its active counterpart "<<active->getKey());
  details::restrict_bases(active, tok);
  // constraint_engine()->propagate();
  if( tok->isFact() )
  active->makeFact();
  if( active->canBeCommitted() )
  m_completed.insert(active);
  if( master.isNoId() ) {
  debugMsg("trex:archive", "Discarding the redundant token "
  <<tok->getPredicateName().toString()
  <<'('<<tok->getKey()<<").");
  discard(tok);
  cli->cancel(tok);
  tok->discard();
  // constraint_engine()->propagate();
  debugMsg("trex:archive", EUROPA::PlanDatabaseWriter::toString(plan_db()));

  ++deleted;
  }
  } else {
  debugMsg("trex:archive", "Cannot commit "<<tok->getPredicateName().toString()
  <<'('<<tok->getKey()<<") as its master "<<master->getPredicateName().toString()
  <<'('<<master->getKey()<<") is not terminated yet.");
  }
  }
  }

  debugMsg("trex:archive", "Evaluating "<<m_committed.size()<<" committed tokens.");
  m_iter = m_committed.begin();

  while( m_committed.end()!=m_iter ) {
  EUROPA::TokenId tok = *(m_iter++);
  EUROPA::TokenSet slaves_t = tok->slaves();
  bool can_delete = true;

  for(EUROPA::TokenSet::const_iterator t=slaves_t.begin();
  slaves_t.end()!=t; ++t) {
  if( !(*t)->isCommitted() ) {
  if( (*t)->end()->lastDomain().getUpperBound()<=now() ) {
  if( (*t)->canBeCommitted() ) {
  debugMsg("trex:archive", "Committing slave "
  <<(*t)->getPredicateName().toString()<<'('<<(*t)->getKey()
  <<") from master "<<tok->getPredicateName().toString()<<'('
  <<tok->getKey()<<").");
  // details::restrict_bases(*t);
  // constraint_engine()->propagate();
  (*t)->commit();
  } else if( (*t)->isMerged() ) {
  EUROPA::TokenId active = (*t)->getActiveToken();
  if( active->canBeCommitted() ) {
  debugMsg("trex:archive", "Collapsing explained slave "
  <<(*t)->getPredicateName().toString()<<'('
  <<(*t)->getKey()
  <<") with its active counterpart "<<active->getKey());
  //details::restrict_bases(active, *t);
  // constraint_engine()->propagate();
  active->commit();
  } else {
  debugMsg("trex:archive", "Collapsing explained slave "
  <<(*t)->getPredicateName().toString()<<'('
  <<(*t)->getKey()
  <<") with its committed active counterpart "
  <<active->getKey());
  //details::restrict_bases(active, *t);
  // constraint_engine()->propagate();
  }
  } else if( !(*t)->isInactive() ) {
  debugMsg("trex:archive", "Cannot delete "<<
  tok->getPredicateName().toString()<<'('<<tok->getKey()
  <<") as one of its slave is not active,merged or "
  <<"inactive ???");
  can_delete = false;
  }
  } else {
  bool protect = true;
  if( (*t)->isMerged() ) {
  EUROPA::TokenId active = (*t)->getActiveToken();

  if( active->start()->lastDomain().getUpperBound()<now() ) {
  EUROPA::ObjectDomain const &dom = active->getObject()->lastDomain();
  EUROPA::ObjectId obj = dom.makeObjectList().front();

  debugMsg("trex:archive", "Checking if object "<<obj->toString()
  <<" of slave "<<(*t)->getKey()<<" is only an Observe timeline.");

  state_iterator i = m_agent_timelines.find(obj->getKey());
  if( m_agent_timelines.end()!=i ) {
  protect = !((*i)->external() && 0==look_ahead(obj));
  }
  }
  }
  if( protect ) {
  debugMsg("trex:archive", "Cannot delete "<<
  tok->getPredicateName().toString()<<'('<<tok->getKey()
  <<") as one of its slave ("<<(*t)->getKey()<<") is not completed");
  can_delete = false;
  }
  }
  }
  }

  EUROPA::TokenSet merged_t = tok->getMergedTokens();
  merged_t.insert(tok);

  for(EUROPA::TokenSet::const_iterator t=merged_t.begin();
  merged_t.end()!=t; ++t) {
  EUROPA::TokenId master = (*t)->master();
  bool restrict = false;
  if( master.isId() ) {
  if( master->end()->lastDomain().getUpperBound()<=now() ) {
  if( master->isCommitted() )
  restrict = true;
  else
  m_completed.insert(master);
  } else {
  debugMsg("trex:archive", "Cannot delete "<<
  tok->getPredicateName().toString()<<'('<<tok->getKey()
  <<") as one of its masters is not completed");
  can_delete = false;
  }
  } else {
  restrict = true;
  }
  if( restrict )
  details::restrict_bases(tok, *t);
  }
  if( can_delete ) {
  debugMsg("trex:archive", "Archiving "<<
  tok->getPredicateName().toString()<<'('<<tok->getKey()
  <<") all its slaves and masters are committed");

  discard(tok);
  EUROPA::TokenSet slaves = tok->slaves();
  for(EUROPA::TokenSet::iterator i=slaves.begin(); slaves.end()!=i; ++i)
  if( (*i)->isMerged() ) {
  // details::restrict_attributes((*i)->getActiveToken(), *i);
  // constraint_engine()->propagate();
  cli->cancel(*i);
  }
  cli->cancel(tok);
  // constraint_engine()->propagate();
  tok->discard();
  debugMsg("trex:archive", EUROPA::PlanDatabaseWriter::toString(plan_db()));
  ++deleted;
  }
  }


  debugMsg("trex:archive", "Archived "<<deleted<<" token(s)");
  debugMsg("trex:archive",
  '['<<now()<<"] ============= END archiving ============");

*/
