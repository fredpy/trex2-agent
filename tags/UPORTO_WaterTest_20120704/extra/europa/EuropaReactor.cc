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
#include "EuropaReactor.hh"
#include "bits/europa_convert.hh"
#include "core/private/CurrentState.hh"

#include <trex/utils/chrono_helper.hh>

#include <PLASMA/Timeline.hh>
#include <PLASMA/Token.hh>
#include <PLASMA/TokenVariable.hh>

#include <PLASMA/PlanDatabaseWriter.hh>

#include <boost/scope_exit.hpp>

// define Europa_Archive_OLD

using namespace TREX::europa;
using namespace TREX::transaction;
using namespace TREX::utils;

namespace {
  std::string const implicit_var("implicit_var_");
}



/*
 * class TREX::europa::EuropaReactor
 */

// structors

EuropaReactor::EuropaReactor(TeleoReactor::xml_arg_type arg)
  :TeleoReactor(arg, false),
   Assembly(parse_attr<std::string>(xml_factory::node(arg), "name")) {
  bool found;
  std::string nddl;

  boost::property_tree::ptree::value_type &cfg = xml_factory::node(arg);
  boost::optional<std::string>
    model = parse_attr< boost::optional<std::string> >(cfg, "model");

  // Load the specified model
  if( model ) {
    if( model->empty() )
      throw XmlError(cfg, "Attribute \"model\" is empty.");
    nddl = *model;
    if( !locate_nddl(nddl) )
      throw XmlError(cfg, "Unable to locate model file \""+(*model)+"\"");
  } else {
    std::string short_nddl = getName().str()+".nddl",
      long_nddl = getGraphName().str()+"."+short_nddl;

    syslog(null, info)<<"No model specified: attempting to load "<<long_nddl;
    nddl = long_nddl;
    if( !locate_nddl(nddl) ) {
      syslog(null, info)<<long_nddl<<" not found: attempting to load "
			<<short_nddl;
      nddl = short_nddl;
      if( !locate_nddl(nddl) )
	throw ReactorException(*this, "Unable to locate "+long_nddl+" or "+short_nddl);
    }
  }
  // Load the nddl model
  if( !playTransaction(nddl) ) {
    syslog(null,error)<<"Model is inconsistent.";
    throw ReactorException(*this, "model in "+nddl+" is inconsistent.");
  }

  if( !plan_db()->isClosed() ) {
    syslog(null, warn)<<"Plan database is not closed:\n\tClosing it now!!!";
    plan_db()->close();
  }

  // Getting planner configuration
  std::string attr = "plan_cfg", planner_cfg, synch_cfg;
  boost::optional<std::string> tmp = parse_attr< boost::optional<std::string> >(cfg, attr);
 
  if( !tmp ) {
    syslog(null, warn)<<"Did not find planner_cfg attribute. Looking for legacy solverConfig instead.";
    attr = "solverConfig";
    tmp = parse_attr< boost::optional<std::string> >(cfg, attr);
    if( !tmp )
      throw XmlError(cfg, "Missing plan_cfg file attribute");
  }
  if( tmp->empty() ) {
    syslog(null, error)<<"Planner config file name is empty.";
    throw XmlError(cfg, "Attribute "+attr+" is not a valid file name.");
  }
  planner_cfg = manager().use(*tmp, found);
  if( !found ) {
    syslog(null, error)<<"Unable to locate planner cfg file \""<<*tmp<<"\"";
    throw ReactorException(*this, "Unable to locate planner cfg \""+(*tmp)+"\"");
  }
  // Getting synchronizer configuration
  tmp = parse_attr< boost::optional<std::string> >(cfg, "synch_cfg");
  if( !tmp ) {
    syslog(null, warn)<<"Did not find synch_cfg attribute. Will use plan_cfg instead.";
    synch_cfg = planner_cfg;
  } else {
    synch_cfg = manager().use(*tmp, found);
    if( !found ) {
      syslog(null, error)<<"Unable to locate synch cfg file \""<<*tmp<<"\"";
      throw ReactorException(*this, "Unable to locate synch cfg \""+(*tmp)+"\"");
    }
  }
    
  try {
    configure_solvers(synch_cfg, planner_cfg);
  } catch(std::exception const &e) {
    syslog(null, error)<<" exception during solvers configuration: "<<e.what();
    throw;
  }

  // Create reactor connections
  std::list<EUROPA::ObjectId> objs;

  trex_timelines(objs);
  if( !objs.empty() ) {
    syslog(null, info)<<"Found "<<objs.size()<<" TREX "<<TREX_TIMELINE.toString()
		      <<" declarations.";
    for(std::list<EUROPA::ObjectId>::const_iterator o=objs.begin();
	objs.end()!=o; ++o) {
      EUROPA::LabelStr name = (*o)->getName(), mode_val;
      Symbol trex_name(name.c_str());
      EUROPA::ConstrainedVariableId o_mode = mode(*o);
      
      if( !o_mode->lastDomain().isSingleton() )
	throw ReactorException(*this, "The mode of the "+TREX_TIMELINE.toString()
			       +" \""+trex_name.str()+"\" is not a singleton.");
      else {
	mode_val = o_mode->lastDomain().getSingletonValue();
      }
      
      if( EXTERNAL_MODE==mode_val || OBSERVE_MODE==mode_val ) {
	use(trex_name, OBSERVE_MODE!=mode_val, with_plan(*o));
	add_state_var(*o);
      } else if( INTERNAL_MODE==mode_val ) {
	provide(trex_name, true, with_plan(*o));
	add_state_var(*o);
      } else if( IGNORE_MODE==mode_val ) {
	ignore(*o);
      } else {
	if( PRIVATE_MODE!=mode_val )
	  syslog(null, warn)<<TREX_TIMELINE.toString()<<" "<<trex_name
			    <<" mode \""
			    <<mode_val.toString()<<"\" is unknown!!!\n"
			    <<"\tI'll assume it is "<<PRIVATE_MODE.toString();
      }
    }
  } else
    syslog(null, warn)<<"No TREX "<<TREX_TIMELINE.toString()
		      <<" found in the model.";
  m_stats.open(file_name("europa_stat.csv").c_str());
  m_stats<<"tick , what, dur_ns, tokens, steps, depth\n";
}

EuropaReactor::~EuropaReactor() {
  m_stats.close();
}


// callbacks

//  - TREX transaction callback

void EuropaReactor::notify(Observation const &obs) {
  setStream();

  EUROPA::ObjectId obj = plan_db()->getObject(obs.object().str());
  bool undefined;
  std::string pred = obs.predicate().str();
  EUROPA::TokenId fact = new_obs(obj, pred, undefined);

  if( undefined )
    syslog(null, warn)<<"Predicate "<<obs.object()<<"."<<obs.predicate()
		      <<" is unknown"<<"\n\t Created "<<pred<<" instead.";
  else if( !restrict_token(fact, obs) )
    syslog(null, error)<<"Failed to restrict some attributes of observation "
		       <<obs;
}

void EuropaReactor::handleRequest(goal_id const &request) {
  setStream();

  EUROPA::ObjectId obj = plan_db()->getObject(request->object().str());
  std::string pred = request->predicate().str();

  if( !have_predicate(obj, pred) ) {
    syslog(null, error)<<"Ignoring Unknow token type "<<request->object()<<'.'
		       <<request->predicate();
  } else {
    // Create the new fact
    EUROPA::TokenId goal = create_token(obj, pred, false);

    if( !restrict_token(goal, *request) ) {
      syslog(null, error)<<"Failed to restrict some attributes of request "
			 <<*request<<"\n\t rejecting it.";
      goal->discard();
    } else {
      // The goal appears to be correct so far : add it to my set of goals
      syslog(info)<<"Integrated request "<<request
		  <<" as the token with Europa ID "
		  <<goal->getKey();
      debugMsg("trex:request", "New goal:\n"<<goal->toLongString());

      m_active_requests.insert(goal_map::value_type(goal->getKey(), request));
      if( m_completed_this_tick ) {
        debugMsg("trex:resume", "[ "<<now()<<"] Resume deliberation due to a request.");
        m_completed_this_tick = false;
      }
    }
  }
}

void EuropaReactor::handleRecall(goal_id const &request) {
  setStream();
  // Remove the goal if it exists
  goal_map::right_iterator i = m_active_requests.right.find(request);
  if( m_active_requests.right.end()!=i ) {
    EUROPA::eint key = i->second;
    m_active_requests.right.erase(i);
    
    syslog(info)<<"Cancel europa goal "<<key<<" due to recall ["<<request<<"]";
    
    recalled(EUROPA::Entity::getTypedEntity<EUROPA::Token>(key));
    if( m_completed_this_tick ) {
      debugMsg("trex:resume", 
	       "[ "<<now()<<"] Resume deliberation due to a recall.");
      m_completed_this_tick = false;
    }
  }
}

void EuropaReactor::newPlanToken(goal_id const &t) {
  syslog(info)<<"Receive token ["<<t<<"] on timeline "<<t->object();
  // treat it as a request for now
  handleRequest(t);
}

void EuropaReactor::cancelledPlanToken(goal_id const &t) {
  syslog(info)<<"Receive cancel for token ["<<t<<"]";
  // treat it as a recall for now
  handleRecall(t);
}


// TREX execution callbacks
void EuropaReactor::handleInit() {
  setStream();
  {
    init_clock_vars();
  }
}

void EuropaReactor::handleTickStart() {
  setStream();
  // Updating the clock
  clock()->restrictBaseDomain(EUROPA::IntervalIntDomain(now(), final_tick()));
  new_tick();
}

bool EuropaReactor::dispatch(EUROPA::TimelineId const &tl,
                             EUROPA::TokenId const &tok) {
  if( m_dispatched.left.find(tok->getKey())==m_dispatched.left.end() ) {
    TREX::utils::Symbol name(tl->getName().toString());
    Goal my_goal(name, tok->getUnqualifiedPredicateName().toString());
    std::vector<EUROPA::ConstrainedVariableId> const &attrs = tok->parameters();

    // Get start, duration and end
    std::auto_ptr<DomainBase>
      d_start(details::trex_domain(tok->start()->lastDomain())),
      d_duration(details::trex_domain(tok->duration()->lastDomain())),
      d_end(details::trex_domain(tok->end()->lastDomain()));

    my_goal.restrictTime(*dynamic_cast<IntegerDomain *>(d_start.get()),
                         *dynamic_cast<IntegerDomain *>(d_duration.get()),
                         *dynamic_cast<IntegerDomain *>(d_end.get()));

    // Manage other attributes
    for(std::vector<EUROPA::ConstrainedVariableId>::const_iterator a=attrs.begin();
        attrs.end()!=a; ++a) {
      // Exclude "implicit_var_*"
      if( 0!=(*a)->getName().toString().compare(0, implicit_var.length(), 
						implicit_var) ) {
	std::auto_ptr<DomainBase> dom(details::trex_domain((*a)->lastDomain()));
	Variable attr((*a)->getName().toString(), *dom);
	my_goal.restrictAttribute(attr);
      }
    }
    goal_id request = postGoal(my_goal);
    syslog(info)<<"Request ["<<request<<"] created from europa token "
		<<tok->getKey();
    if( request ) {
      m_dispatched.insert(goal_map::value_type(tok->getKey(), request));
    } else
      return false;
  }
  return true;
}

void EuropaReactor::plan_dispatch(EUROPA::TimelineId const &tl, EUROPA::TokenId const &tok)
{
  if( m_plan_tokens.left.find(tok->getKey()) == m_plan_tokens.left.end() ) {
    TREX::utils::Symbol name(tl->getName().toString());
    Goal my_goal(name, tok->getUnqualifiedPredicateName().toString());
    restrict_goal(my_goal, tok);

    goal_id request = postPlanToken(my_goal);
    if( request )
      m_plan_tokens.insert(goal_map::value_type(tok->getKey(), request));
  }
  else {
    restrict_goal(*(m_plan_tokens.left.at(tok->getKey())),tok);
  }
}

void EuropaReactor::restrict_goal(Goal& goal, EUROPA::TokenId const &tok)
{
    std::vector<EUROPA::ConstrainedVariableId> const &attrs = tok->parameters();
    std::auto_ptr<DomainBase>
        d_start(details::trex_domain(tok->start()->lastDomain())),
        d_duration(details::trex_domain(tok->duration()->lastDomain())),
        d_end(details::trex_domain(tok->end()->lastDomain()));

    goal.restrictTime(*dynamic_cast<IntegerDomain *>(d_start.get()),
                       *dynamic_cast<IntegerDomain *>(d_duration.get()),
                       *dynamic_cast<IntegerDomain *>(d_end.get()));

    // Manage other attributes
    for(std::vector<EUROPA::ConstrainedVariableId>::const_iterator a=attrs.begin();
        attrs.end()!=a; ++a) {
      // ignore implicit_var
      if( 0!=(*a)->getName().toString().compare(0, implicit_var.length(), 
						implicit_var)) {
	std::auto_ptr<DomainBase> dom(details::trex_domain((*a)->lastDomain()));
	Variable attr((*a)->getName().toString(), *dom);
	goal.restrictAttribute(attr);
      }
    }
}

void EuropaReactor::print_stats(std::string const &what, 
				size_t steps, size_t depth,
				EuropaReactor::stat_clock::duration const &dur) {
  m_stats<<now()<<", "<<what<<", "<<dur.count()
	 <<", "<<plan_db()->getTokens().size()
	 <<", "<<steps<<", "<<depth<<std::endl;
}


bool EuropaReactor::do_relax(bool full) {
  logPlan("failed");
  stat_clock::time_point start = stat_clock::now();
  bool ret = relax(full);
  print_stats("relax", 0, 0, stat_clock::now()-start);
  logPlan("relax");
  return ret;
}

bool EuropaReactor::synch() {
  stat_clock::time_point start = stat_clock::now();
  bool ret = do_synchronize();
  print_stats("synch", synchronizer()->getStepCount(), 
	      synchronizer()->getDepth(), 
	      stat_clock::now()-start);
  return ret;
}

bool EuropaReactor::synchronize() {
  setStream();
  EuropaReactor &me = *this;
  debugMsg("trex:synch", "["<<now()<<"] BEGIN synchronization =====================================");
  me.logPlan("tick");
  BOOST_SCOPE_EXIT((&me)) {
    me.synchronizer()->clear();
    me.logPlan("synch");
    debugMsg("trex:synch", "["<<me.now()<<"] END synchronization =======================================");
    debugMsg("trex:synch", "Plan after synchronization:\n"
             <<EUROPA::PlanDatabaseWriter::toString(me.plan_db()));
//        debugMsg("trex:synch", "Detailed decision stack:\n"<<oss.str());
  } BOOST_SCOPE_EXIT_END;

  tr_info("resolve state");
  if( !synch() ) {
    m_completed_this_tick = false;
    syslog(null, warn)<<"Failed to synchronize : relaxing current plan.";

    if( !( do_relax(false) && synch() ) ) {
      syslog(null, warn)<<"Failed to synchronize(2) : forgetting past.";
      if( !( do_relax(true) && synch() ) ) {
        syslog(null, error)<<"Failed to synchronize(3) : killing reactor";
        return false;
      }
    }
  } else { // things to do when everything went fine:

    // Prepare the reactor for next deliberation round 
    if( m_completed_this_tick ) {
      tr_info("clean-up plan solver");
      planner()->clear(); // remove the past decisions of the planner

      tr_info("remove completed requests");
      Assembly::external_iterator from(begin(), end()), to(end(), end());
      for( ; to!=from; ++from) {
	EUROPA::TokenId cur = (*from)->previous();
	if( cur.isId() && cur->isMerged() )
	  m_dispatched.left.erase(cur->getActiveToken()->getKey());
      }
      m_completed_this_tick = false;
    } 
#ifndef Europa_Archive_OLD
    tr_info("archive");
    // Necessary in case the planner did not run on previous tick
    if( planner()->getStepCount()==0 ) {
      stat_clock::time_point start = stat_clock::now();
      archive();
      print_stats("archive", 0, 0, stat_clock::now()-start);
    }
#endif 
  }
  tr_info("end of synch");
  return constraint_engine()->propagate(); // should not fail
}

bool EuropaReactor::discard(EUROPA::TokenId const &tok) {
  goal_map::left_iterator i = m_active_requests.left.find(tok->getKey());
  bool ret = false;

  if( m_active_requests.left.end()!=i ) {
    syslog(null, info)<<"Discarded completed request ["<<i->second<<"]";
    m_active_requests.left.erase(i);
    ret = true;
  }
  i = m_dispatched.left.find(tok->getKey());
  if( m_dispatched.left.end()!=i ) {
    // syslog(null, info)<<"Discarded past goal ["<<i->second<<"]";
    m_dispatched.left.erase(i);
    ret = true;
  }
  i = m_plan_tokens.left.find(tok->getKey());
  if( m_plan_tokens.left.end()!=i ) {
    m_plan_tokens.left.erase(i);
    ret = true;
  }
  return ret;
}

void EuropaReactor::cancel(EUROPA::TokenId const &tok) {
  goal_map::left_iterator i = m_dispatched.left.find(tok->getKey());

  if( m_dispatched.left.end()!=i ) {
    syslog(info)<<"Recall ["<<i->second<<"]";
    postRecall(i->second);
    m_dispatched.left.erase(i);
  }

  i = m_plan_tokens.left.find(tok->getKey());
  if( m_plan_tokens.left.end()!=i ) {
    cancelPlanToken(i->second);
    m_plan_tokens.left.erase(i);
  }
}

bool EuropaReactor::hasWork() {
  setStream();
  if( constraint_engine()->provenInconsistent() ) {
    syslog(null, error)<<"Plan database is inconsistent.";
    return false;
  }
  if( planner()->isExhausted() ) {
    syslog(null, warn)<<"Deliberation solver is exhausted.";
    return false;
  }
  if( !m_completed_this_tick ) {
    if( planner()->noMoreFlaws() ) {
      size_t steps = planner()->getStepCount();
      m_completed_this_tick = true;
#ifdef Europa_Archive_OLD
      { // mesure archiving time 
        stat_clock::time_point start = stat_clock::now();
        planner()->clear();
        archive();
	print_stats("archive", 0, 0, stat_clock::now()-start);
      }
#endif // Europa_Archive_OLD
      debugMsg("trex:resume", "[ "<<now()<<"] Deliberation completed after "<<steps<<" steps.");
      if( steps>0 ) {
        syslog(null, info)<<"Deliberation completed in "<<steps<<" steps.";
        logPlan("plan");
        getFuturePlan();
      }
      Assembly::external_iterator from(begin(), end()), to(end(), end());
      for(; to!=from; ++from) {
        TeleoReactor::external_iterator
        j=find_external((*from)->timeline()->getName().c_str());
        EUROPA::eint e_lo, e_hi;
        if( j.valid() && j->accept_goals() ) {
          IntegerDomain window = j->dispatch_window(getCurrentTick()+1);
          IntegerDomain::bound lo = window.lowerBound(), 
	    hi = window.upperBound();
          e_lo = static_cast<EUROPA::eint::basis_type>(lo.value());
          if( hi.isInfinity() )
            e_hi = final_tick();
          else
            e_hi = static_cast<EUROPA::eint::basis_type>(hi.value());
          (*from)->do_dispatch(e_lo, e_hi);
        }
      }
    }
  }  
  return !m_completed_this_tick;
}

void EuropaReactor::resume() {
  setStream();

  stat_clock::time_point start = stat_clock::now();

  if( constraint_engine()->pending() )
    constraint_engine()->propagate();

  if( constraint_engine()->constraintConsistent() )
    planner()->step();

  bool should_relax = false;

  if( constraint_engine()->provenInconsistent() ) {
    syslog(null, warn)<<"Inconsitency found during planning.";
    should_relax = true;
  }
  if( planner()->isExhausted() ) {
    syslog(null, warn)<<"Deliberation solver is exhausted.";
    should_relax = true;
  }

  if( should_relax ) {
    syslog(null, warn)<<"Relax database after "<<planner()->getStepCount()
		      <<" steps.";
    if( !do_relax(false) )
      syslog(null, warn)<<"Failed to relax => forgetting past.";
      if( !do_relax(true) ) {
        syslog(null, error)<<"Unable to recover from plan inconsistency.";
	throw TREX::transaction::ReactorException(*this, "Unable to recover from plan inconsistency.");
      }
  }
  print_stats("delib", planner()->getStepCount(), 
	      planner()->getDepth(), stat_clock::now()-start);
}

// europa core callbacks

void EuropaReactor::notify(EUROPA::LabelStr const &object,
			   EUROPA::TokenId const &tok) {
  Observation obs(object.c_str(),
		  tok->getUnqualifiedPredicateName().toString());

  std::vector<EUROPA::ConstrainedVariableId> const &attr = tok->parameters();

  for(std::vector<EUROPA::ConstrainedVariableId>::const_iterator a=attr.begin();
      attr.end()!=a; ++a) {
    // ignore implicit_var
    if( 0!=(*a)->getName().toString().compare(0, implicit_var.length(), 
					      implicit_var) ) {
      std::auto_ptr<TREX::transaction::DomainBase>
	dom(details::trex_domain((*a)->lastDomain()));
      TREX::transaction::Variable var((*a)->getName().toString(), *dom);
      obs.restrictAttribute(var);
    }
  }
  postObservation(obs);
}


// manipulators

bool EuropaReactor::restrict_token(EUROPA::TokenId &tok,
				   Predicate const &pred) {
  bool no_empty = true;
  std::list<Symbol> attrs;
  pred.listAttributes(attrs, false);

  for(std::list<Symbol>::const_iterator v=attrs.begin(); attrs.end()!=v; ++v) {
    EUROPA::ConstrainedVariableId param = tok->getVariable(v->str());

    if( param.isId() ) {
      Variable const &var = pred[*v];
      //syslog("INFO")<<"Apply "<<tok->toString()<<"."<<var;
      try {
	details::europa_restrict(param, var.domain());
      } catch(DomainExcept const &e) {
	syslog(null, warn)<<"Failed to restrict attribute "<<(*v)
		      <<" on token "<<pred.object()<<'.'<<pred.predicate()
		      <<": "<<e;
	no_empty = false;
      }
    } else
      syslog(null, warn)<<" Ignoring unknown attribute "<<pred.object()
		    <<'.'<<pred.predicate()<<'.'<<(*v);
  }
  return no_empty;
}

// Observers

EUROPA::edouble EuropaReactor::tick_to_date(EUROPA::eint tick) const {
  typedef chrono_posix_convert< boost::chrono::duration<EUROPA::edouble::basis_type> > convert;
  return convert::to_chrono(tickToTime(EUROPA::cast_basis(tick))-boost::posix_time::from_time_t(0)).count();
}

EUROPA::eint EuropaReactor::date_to_tick(EUROPA::edouble date) const {
  typedef chrono_posix_convert< boost::chrono::duration<EUROPA::edouble::basis_type> > convert;
  convert::chrono_duration rdate(EUROPA::cast_basis(date));
  return static_cast<EUROPA::eint::basis_type>(timeToTick(boost::posix_time::from_time_t(0)+convert::to_posix(rdate)));
}

EUROPA::IntervalIntDomain EuropaReactor::plan_scope() const {
  EUROPA::eint scope_duration(static_cast<EUROPA::eint::basis_type>(getExecLatency()+getLookAhead()));
  return EUROPA::IntervalIntDomain(now(), std::min(now()+scope_duration,
						   final_tick()));
}

void EuropaReactor::logPlan(std::string const &base_name) const {
  LogManager::path_type full_name = file_name(base_name+".gv");
  std::ofstream out(full_name.c_str());
  print_plan(out/*, true*/);
}

