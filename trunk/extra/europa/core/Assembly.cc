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

#include <PLASMA/PlanDatabaseWriter.hh>


#include <PLASMA/Context.hh>

#include <PLASMA/XMLUtils.hh>
#include <PLASMA/Debug.hh>

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

Assembly::Assembly(std::string const &name)
  :m_in_synchronization(false), m_name(name) {
  //  redirect log output to <name>/europa.log
  m_trex_schema->setStream(m_debug, m_name+"/europa.log");

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

  m_cstr_engine->setAutoPropagation(true);

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

  if( tok->canBeCommitted() )
    tok->commit();
  else if( !tok->isCommitted() )
    m_completed.insert(tok);

  // I assume here that if a goals ends time is in the past then the goal is completed

  if( tok->isMerged() ) {
    active = tok->getActiveToken();
    if( rejectable(active) && discard(active) ) {
      details::restrict_base(active, active->end(), tok->end()->baseDomain());
      details::restrict_base(active, active->start(), tok->start()->baseDomain());
      if( !active->isCommitted() )
        m_completed.insert(active);
    }
  }
  for(EUROPA::TokenSet::const_iterator i=active->getMergedTokens().begin();
      active->getMergedTokens().end()!=i; ++i) {
    if( rejectable(*i) && discard(*i) ) {
      details::restrict_base(*i, (*i)->end(), tok->end()->baseDomain());
      details::restrict_base(*i, (*i)->start(), tok->start()->baseDomain());
      if( !(*i)->isCommitted() )
        m_completed.insert(*i);
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
  std::auto_ptr<EUROPA::TiXmlElement>
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
  m_in_synchronization = true;

  BOOST_SCOPE_EXIT((&m_in_synchronization)) {
    m_in_synchronization = false;
  } BOOST_SCOPE_EXIT_END

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
  } BOOST_SCOPE_EXIT_END


  if( aggressive )
    relax_name = "AGGRESSIVE "+relax_name;

  debugMsg("trex:relax", "["<<now()<<"] =================== START "<<relax_name<<" =================");
  // Clean up decisions made by solvers
  debugMsg("trex:relax", "Cancelling current decisions");
  synchronizer()->reset();
  planner()->reset();

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
      debugMsg("trex:relax", "\t- "<<tok->getKey()<<" necessarily ends in the past");
      if( tok->isFact() ) {
        debugMsg("trex:relax", "\t- "<<tok->getKey()<<" is a fact");
        if( aggressive ) {
          debugMsg("trex:relax", "\t- destroying "<<tok->getKey()<<" (aggressive)");
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
        }
      } else {
        bool can_reject = rejectable(tok);
        
        if( can_reject || aggressive ) {
          if( can_reject ) {
            debugMsg("trex:relax", "\t- destroying the goal "<<tok->getKey());
          } else {
            debugMsg("trex:relax", "\t- destroying past token "<<tok->getKey()<<" (aggressive)");
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
          cli->cancel(tok->getActiveToken());
          if( !aggressive ) {
            debugMsg("trex:relax", "\t- activate fact "<<tok->getKey());
            cli->activate(tok);
          }
        }
      } else if( !tok->isInactive() ) {
        debugMsg("trex:relax", "\t- cancelling non fact "<<tok->getKey());
        cli->cancel(tok);
      }
    }
  }

  debugMsg("trex:relax", "["<<now()<<"] =================== END "<<relax_name<<" =================");
  bool ret = constraint_engine()->propagate();
  if( !ret )
    debugMsg("trex:relax", "RELAX FAILURE !!!!");
  return ret;
}

void Assembly::archive() {
#ifdef TREX_ARCHIVE_Greedy

# ifdef EUROPA_HAVE_EFFECT
  /*
   * As of now Greedy works badly with actions on 2.6. It is not really able to
   * get rid of them when they are not part of an Internal or External timeline
   * What is needed is to annalyze for efficeintly for each action :
   *
   * If all the conditions are committed and all the effects completed then the
   * action is completed ... but how do we consider relations that are not
   * conditions or actions ?
   */
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
      debugMsg("trex:archive", "Committing "<<tok->getPredicateName().toString()
	       <<'('<<tok->getKey()<<").");
      details::restrict_bases(tok);
      // constraint_engine()->propagate();
      tok->commit(); 
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
	} else if( master->canBeCommitted() ) {
	  details::restrict_bases(master);
	  // constraint_engine()->propagate();
	  master->commit();
	  debugMsg("trex:archive", "Ignoring inactive freshly justified token "
		   <<tok->getPredicateName().toString()
		   <<'('<<tok->getKey()<<").");
	  m_completed.erase(tok);
	} 
      }	
    } else {
      // token is merged
      EUROPA::TokenId active = tok->getActiveToken();
      EUROPA::TokenId master = tok->master();

      if( master.isId() ) {
	if( master->end()->lastDomain().getUpperBound()<=now() ) {
	  if( master->canBeCommitted() ) {
	    details::restrict_bases(master);
	    // constraint_engine()->propagate();
	    master->commit();
	  }
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
	  active->commit();
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
            details::restrict_bases(*t);
	    // constraint_engine()->propagate();
            (*t)->commit();
          } else if( (*t)->isMerged() ) {
	    EUROPA::TokenId active = (*t)->getActiveToken();
	    if( active->canBeCommitted() ) {
	      debugMsg("trex:archive", "Collapsing explained slave "
		       <<(*t)->getPredicateName().toString()<<'('
		       <<(*t)->getKey()
		       <<") with its active counterpart "<<active->getKey());
	      details::restrict_bases(active, *t);
	      // constraint_engine()->propagate();
	      active->commit();
	    } else {
	      debugMsg("trex:archive", "Collapsing explained slave "
		       <<(*t)->getPredicateName().toString()<<'('
		       <<(*t)->getKey()
		       <<") with its committed active counterpart "
		       <<active->getKey());
	      details::restrict_bases(active, *t);
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
	  debugMsg("trex:archive", "Cannot delete "<<
		   tok->getPredicateName().toString()<<'('<<tok->getKey()
		   <<") as one of its slave is not completed");
          can_delete = false;
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
	  if( master->canBeCommitted() ) {
	    details::restrict_bases(master);
	    // constraint_engine()->propagate();
	    master->commit();
	  }
	  restrict = true;
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

#else // TREX_ARCHIVE_EuropaDefault

  // Just rely on the europa archival : safe but inefficient
  plan_db()->archive(now()-1);

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
    if( is_action(*it) )
      out<<"ACTION";
    else if( is_predicate(*it) )
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
    if( (*it)->isCommitted() )
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
        if( is_effect(*t) )
          out<<"\\n(effect)";
        if( is_condition(*t) )
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

void Assembly::backtracking(EUROPA::SOLVERS::DecisionPointId &dp) {
  debugMsg("trex:always", "["<<now()<<"] Last decision : "<<m_synchronizer->getLastExecutedDecision());
}

void Assembly::print_context(std::ostream &out, EUROPA::ConstrainedVariableId const &v) const {
  EUROPA::TokenId tok = details::parent_token(v);
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
  debugMsg("trex:synch:search", "New decision point ["<<dp->getKey()<<"]:" <<dp->toString());
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
  EUROPA::TokenId master = token->master();

  if( master.isNoId() ) {
    m_owner.m_roots.insert(token);
  }
}

void Assembly::listener_proxy::notifyRemoved(EUROPA::TokenId const &token) {
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
    // Check if the token is a goal and add it to m_goals
    if(m_owner.is_goal(token)) {
        m_owner.m_goals.insert(token);
        // Checking for goal->effect->action relation if it is then the action
        // becomes an action goal in the m_goals
        if(m_owner.actionEffect(token))
            m_owner.m_goals.insert(token->master());
    }
}

void Assembly::listener_proxy::notifyDeactivated(EUROPA::TokenId const &token) {
  // Checks and erases the token if it was considered a goal
  m_owner.m_goals.erase(token);

  if( m_owner.is_agent_timeline(token) ) {
    debugMsg("trex:archive", "cancel "<<token->getPredicateName().toString()
	     <<'('<<token->getKey()<<')');
    m_owner.cancel(token);
  }
}

void Assembly::listener_proxy::notifyMerged(EUROPA::TokenId const &token)
{
    // Check to find if the tokens active token is in m_goal
    // If it is it gets added to m_goals
    if(m_owner.m_goals.find(token->getActiveToken())!=m_owner.m_goals.end())
    {
        m_owner.m_goals.insert(token);
        // Checking for goal->effect->action relation if it is then the action
        // becomes an action goal in the m_goals
        if(m_owner.actionEffect(token))
            m_owner.m_goals.insert(token->master());
    }
    // Check if the token is a goal and adds it to m_goals
    // Also insert the active token as that is the token we most often check against
    else if(m_owner.is_goal(token)) {
        m_owner.m_goals.insert(token);
        m_owner.m_goals.insert(token->getActiveToken());
        // Checking for goal->effect->action relation if it is then the action
        // becomes an action goal in the m_goals
        if(m_owner.actionEffect(token))
            m_owner.m_goals.insert(token->master());
        if(m_owner.actionEffect(token->getActiveToken()))
            m_owner.m_goals.insert(token->getActiveToken()->master());
    }
}

void Assembly::listener_proxy::notifySplit(EUROPA::TokenId const &token) {
    // Checks and erases the token if it was considered a goal
    m_owner.m_goals.erase(token);
}

void Assembly::listener_proxy::notifyRejected(EUROPA::TokenId const &token) {

}

void Assembly::listener_proxy::notifyReinstated(EUROPA::TokenId const &token) {

}

void Assembly::listener_proxy::notifyCommitted(EUROPA::TokenId const &token) {
  m_owner.erase(m_owner.m_completed, token);
  m_owner.m_committed.insert(token);
}

void Assembly::listener_proxy::notifyTerminated(EUROPA::TokenId const &token) {

}

//Listener_proxy helper functions
bool Assembly::actionEffect(const EUROPA::TokenId& token)
{
    EUROPA::TokenId master = token->master();
    if(master.isId())
    {
        if(is_action(master) && is_effect(token))
            return true;
    }
    return false;
}

