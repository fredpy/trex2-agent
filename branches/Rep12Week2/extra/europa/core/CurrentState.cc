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
#include "private/CurrentState.hh"
#include "trex/europa/Assembly.hh"
#include "trex/europa/bits/europa_helpers.hh"

#include <PLASMA/TokenVariable.hh>
#include <PLASMA/PlanDatabaseWriter.hh>

using namespace TREX::europa;
using namespace TREX::europa::details;

namespace EUROPA {
  namespace SOLVERS {

    /*
     * Specializations of getMatches* in the europa matching engine in order to
     * be able to manage our new CurrentState flaws
     */
    template<>
    void MatchingEngine::getMatchesInternal(TREX::europa::details::CurrentStateId const &inst,
                                            std::vector<MatchingRuleId> &results) {
      trigger(inst->entityType(), m_rulesByObjectType, results);
    }

    template<>
    void MatchingEngine::getMatches(TREX::europa::details::CurrentStateId const &inst,
                                    std::vector<MatchingRuleId> &results) {
      debugMsg("trex:synch", "Attempt to match CurrentState ("<<inst->timeline()->toString()<<')');
      results = m_unfilteredRules;
      getMatchesInternal(inst, results);
    }

  } // EUROPA::SOLVERS
} // EUROPA


/*
 * struct TREX::europa::details::is_external
 */

bool is_external::operator()(CurrentStateId const &timeline) const {
  return timeline.isId() && timeline->external();
}

/*
 * struct TREX::europa::details::is_internal
 */

bool is_internal::operator()(CurrentStateId const &timeline) const {
  return timeline.isId() && timeline->internal();
}

/*
 * class TREX::europa::details::CurrentState
 */

// structors

CurrentState::CurrentState(Assembly &assembly, EUROPA::TimelineId const &timeline)
  :m_assembly(assembly), m_client(assembly.plan_db()->getClient()),
   m_timeline(timeline), m_id(this) {
  assembly.predicates(timeline, m_pred_names);
  // removed special values
  m_pred_names.erase(Assembly::FAILED_PRED);
  m_pred_names.erase(Assembly::UNDEFINED_PRED);

  // Now extract the default predicate (if any)
  EUROPA::ConstrainedVariableId default_attr = assembly.default_pred(timeline);

  // For now I keep this default behavior that prohibits to have no default
  // It may change in the future
  if( !default_attr->lastDomain().isSingleton() )
    throw EuropaException("Agent timeline "+timeline->getName().toString()+
			  " does not specify its default predicate.");
  m_pred_default = default_attr->lastDomain().getSingletonValue();

  // Now I need to check that this rpedicate is valid
  if( !assembly.schema()->isPredicate(predicate_name(timeline, m_pred_default)) )
    throw EuropaException("Agent timeline "+timeline->getName().toString()+
			  " default predicate \""+m_pred_default.toString()
			  +"\" does not exists.");
  // Finally remove the default from the alternate choices
  m_pred_names.erase(m_pred_default);
}

// observers

bool CurrentState::has_default() const {
  return true; // for now it always have a default
               //  (required at construction ... see above)
}

EUROPA::eint CurrentState::now() const {
  return m_assembly.now();
}



bool CurrentState::identified() const {
  EUROPA::TokenId cur = current();

  if( cur.isId() ) {
    if( cur->isMerged() )
      cur = cur->getActiveToken();
    return cur->end()->lastDomain().getLowerBound() > now();
  }
  return false;
}

bool CurrentState::committed() const {
  return current().isId()
    && m_frontier->baseDomain().getLowerBound() > now();
}

bool CurrentState::check_committed() const {
  return committed() && m_prev_obs.isNoId()
    && current()->start()->isSpecified()
    && current()->start()->getSpecifiedValue()!=now();
}


bool CurrentState::internal() const {
  return m_assembly.internal(EUROPA::ObjectId(m_timeline));
}

bool CurrentState::external() const {
  return m_assembly.external(EUROPA::ObjectId(m_timeline));
}

// modifiers

EUROPA::TokenId CurrentState::new_obs(std::string const &pred,
				      bool insert) {

  debugMsg("trex:state", "["<<now()<<"] Creating new observation "<<pred<<" on "<<timeline()->toString());
  EUROPA::TokenId tok = m_client->createToken(pred.c_str(), NULL, false, true);

  tok->start()->specify(now());
  tok->getObject()->specify(m_timeline->getKey());
  new_token(tok);

  if( insert ) {
    debugMsg("trex:state", "["<<now()<<"] Creating new observation "<<pred
	     <<" on "<<timeline()->toString());
    m_client->activate(m_last_obs);
    if( m_prev_obs.isId() ) {
      EUROPA::TokenId active = m_prev_obs;
      if( m_prev_obs->isMerged() )
        active = m_prev_obs->getActiveToken();
      m_client->constrain(m_timeline, active, m_last_obs);
    }
  }
  return current();
}

void CurrentState::erased(EUROPA::TokenId const &token) {
  if( m_last_obs==token ) {
    m_last_obs = EUROPA::TokenId::noId();
    m_frontier = EUROPA::ConstrainedVariableId::noId();
  } 
  else if( m_prev_obs==token ) {
    m_prev_obs = EUROPA::TokenId::noId();
    m_prev_frontier = EUROPA::ConstrainedVariableId::noId();
  }
}

void CurrentState::apply_base(EUROPA::TokenId &merged,
			      EUROPA::Id<TimePoint> &frontier) {
  EUROPA::TokenId active = merged->getActiveToken();
  std::vector<EUROPA::ConstrainedVariableId> const 
    &actives = active->getVariables();
  std::vector<EUROPA::ConstrainedVariableId> const 
    &mergeds = merged->getVariables();

  for(size_t v=1; v<actives.size(); ++v)
    actives[v]->restrictBaseDomain(mergeds[v]->lastDomain());

  if( frontier.isId() )
    frontier->setToken(active);
  active->incRefCount();
  merged->decRefCount();
  merged = active;
}

void CurrentState::replaced(EUROPA::TokenId const &token) {
  EUROPA::eint t_start;

  if( m_last_obs==token )
    apply_base(m_last_obs, m_frontier);
  else if( m_prev_obs==token )
    apply_base(m_prev_obs, m_prev_frontier);
}


void CurrentState::new_token(EUROPA::TokenId const &token) {
  m_prev_obs = m_last_obs;
  m_prev_frontier = m_frontier;
  m_last_obs = token;
  m_last_obs->incRefCount();
  m_frontier = (new TimePoint(m_last_obs, now(), "__trex_frontier"))->getId();
}

void CurrentState::relax_token() {
  m_client->deleteToken(m_last_obs);
  m_last_obs = m_prev_obs;
  m_frontier = m_prev_frontier;
  m_prev_obs = EUROPA::TokenId::noId();
  m_prev_frontier = EUROPA::TokenId::noId();
}

void CurrentState::push_end() {
  m_frontier->set_date(now());
}

void CurrentState::relax_end() {
  m_frontier->relax_date();
}

bool CurrentState::commit() {
  
  if( check_committed() )
    return true;
  else {
    EUROPA::eint start_time;

    if( m_prev_obs.isId() ) {
      assert(m_prev_frontier.isId());
      debugMsg("trex:commit", "Terminating "<<timeline()->toString()
	       <<'.'<<m_prev_obs->getUnqualifiedPredicateName().toString()
	       <<'('<<m_prev_obs->getKey()<<')');
      if( !m_prev_frontier->commit_end(now()) ) {
        debugMsg("trex:always", "["<<now()<<"] Failed to terminate observation "
                 <<timeline()->toString()<<'.'
                 <<m_prev_obs->getUnqualifiedPredicateName().toString()
                 <<'('<<m_prev_obs->getKey()<<')');
        return false;     
      }        
      m_assembly.terminate(m_prev_obs);
      m_prev_obs = EUROPA::TokenId::noId();
      m_prev_frontier = EUROPA::Id<TimePoint>::noId();
      start_time = now();
    } else
      start_time = m_last_obs->start()->getSpecifiedValue();

    if( now()==start_time ) {
      debugMsg("trex:commit", "Starting "<<timeline()->toString()<<'.'
	       <<m_last_obs->getUnqualifiedPredicateName().toString()
	       <<'('<<m_last_obs->getKey()<<')');
      restrict_base(m_last_obs, m_last_obs->start(), 
		    EUROPA::IntervalIntDomain(now()));
      m_assembly.notify(*this);
    } 
    debugMsg("trex:commit", "Extend duration of "<<timeline()->toString()<<'.'
             <<m_last_obs->getUnqualifiedPredicateName().toString()
             <<'('<<m_last_obs->getKey()<<"):\n  BEFORE\n\tend="
             <<m_last_obs->end()->baseDomain().toString()<<"\n\tduration="
             <<m_last_obs->duration()->baseDomain().toString()<<'\n');   
    if( !m_frontier->commit_date(now()) ) {
      debugMsg("trex:always", "["<<now()
               <<"] Failed to extend external observation "
               <<timeline()->toString()
               <<"."<<m_last_obs->getUnqualifiedPredicateName().toString()
               <<'('<<m_last_obs->getKey()<<").");
      return false;	
    }
  }
  if( m_assembly.constraint_engine()->propagate() )
    return true;
  debugMsg("trex:always", "["<<now()
           <<"] Inconsitency detected after applying observation "
           <<timeline()->toString()<<"."
           <<m_last_obs->getUnqualifiedPredicateName().toString()
           <<'('<<m_last_obs->getKey()<<").");

  return false;
}

// manipulators

void CurrentState::do_dispatch(EUROPA::eint lb, EUROPA::eint ub) {
  std::list<EUROPA::TokenId>::const_iterator
    i = timeline()->getTokenSequence().begin(),
    endi = timeline()->getTokenSequence().end();
  std::string tl = timeline()->toString();

  debugMsg("trex:dispatch", "Checking dispatch for "<<tl<<" ["<<lb<<", "<<ub
	   <<"]\n\t"<<timeline()->getTokenSequence().size()<<" tokens to check.");

  // skip the past tokens
  for( ; endi!=i && (*i)->start()->lastDomain().getUpperBound()<lb && (*i)->end()->lastDomain().getLowerBound()<=lb+1; ++i) {
    debugMsg("trex:dispatch", "skipping "<<tl<<'.'<<(*i)->getUnqualifiedPredicateName().toString()<<'('<<(*i)->getKey()
	     <<") as it ends to early (end="
	     <<(*i)->end()->lastDomain().toString()<<"<="<<(lb+1));
  }
  if( i!=endi )
    debugMsg("trex:dispatch", "First token is "<<tl<<'.'<<(*i)->getUnqualifiedPredicateName().toString()<<'('
	     <<(*i)->getKey()<<") start="<<(*i)->start()->lastDomain().toString());

  for( ; endi!=i && (*i)->start()->lastDomain().getLowerBound()<=ub; ++i) {
#ifdef  EUROPA_HAVE_EFFECT
    debugMsg("trex:dispatch", "Checking if token "<<tl<<'.'<<(*i)->getUnqualifiedPredicateName().toString()
	     <<'('<<(*i)->getKey()<<") overlaps ["<<lb<<", "<<ub
	     <<"] as a goal (start="<<(*i)->start()->lastDomain().toString()); 
    // New code from Philip 
    // Check if next candidate is goal dependent ... needs to be refined I think
    EUROPA::TokenId nowGoal = getGoal(*i, lb, ub);
    if(nowGoal.isId())
      m_assembly.dispatch(timeline(),*i);
#else 
    debugMsg("trex:dispatch", "Checking if token "<<tl<<'.'<<(*i)->getUnqualifiedPredicateName().toString()
	     <<'('<<(*i)->getKey()<<") overlaps ["<<lb<<", "<<ub
	     <<"] (start="<<(*i)->start()->lastDomain().toString()); 
    ///Old code for dispatching tokens
    // very dumb but work with europa 2.5
    if( !m_assembly.dispatch(timeline(), *i)
	&& (*i)->start()->lastDomain().getLowerBound()>=lb ) {
      debugMsg("trex:dispatch", "Token "<<tl<<'.'<<(*i)->getUnqualifiedPredicateName().toString()<<'('
	       <<(*i)->getKey()<<") cannot be dispatched and starts after "
	       <<lb<<"\n\t=>stopping dispatch for "<<timeline()->toString());
      break;
    } else {
      debugMsg("trex:dispatch", "Token "<<tl<<'.'<<(*i)->getUnqualifiedPredicateName().toString()<<'('
	       <<(*i)->getKey()<<") dispatched");
    }
#endif // EUROPA_HAVE_EFFECT
  }
}

EUROPA::TokenId  CurrentState::getGoal(const EUROPA::TokenId& token, EUROPA::eint lb, EUROPA::eint ub)
{
  EUROPA::TokenSet actions, condActions, merged;
  EUROPA::TokenSet::iterator it, end;
  ///Gets all of the tokens merged with @token
  merged = getAllTokens(token);

  ///Tests to see if any of the tokens are fact and then @returns noId()
  ///and gets the masters of all the merged tokens
  for(it = merged.begin(), end = merged.end(); it!=end; it++)
    {
      if((*it)->isFact())
	return EUROPA::Id<EUROPA::Token>::noId();

      EUROPA::TokenId master = (*it)->master();
      if(master.isId())
	{
	  if(m_assembly.is_effect((*it)) && m_assembly.is_action(master))
	    actions.insert(master);
	  else if(m_assembly.is_condition((*it))
		  || (!m_assembly.is_condition((*it)) && !master->isFact()))
	    condActions.insert(master);
	}
    }

  EUROPA::TokenId goal = searchGoal(actions);
  if(goal.isId() && goal->start()->lastDomain().getLowerBound()<=ub)
    return goal;
  else
    {
      goal = searchGoal(condActions);
      if(goal.isId() && goal->start()->lastDomain().getLowerBound()<=ub)
	return goal;
    }
  return EUROPA::Id<EUROPA::Token>::noId();
}

EUROPA::TokenId CurrentState::searchGoal(EUROPA::TokenSet actions)
{
  EUROPA::TokenSet effects,conditions, merged, slaves;
  EUROPA::TokenSet::iterator it, end, mergedIt, slave;
  for(it=actions.begin(), end=actions.end(); it!=end; it++)
    {
      merged = getAllTokens((*it));
      for(mergedIt=merged.begin(); mergedIt!=merged.end(); mergedIt++)
	{
	  slaves = (*mergedIt)->slaves();
	  for(slave=slaves.begin();slave!=slaves.end(); slave++)
	    {
	      if(!m_assembly.is_condition((*slave)))
		effects.insert((*slave));
	      else
		conditions.insert((*slave));
	    }
	}
      std::list<EUROPA::TokenId> search;
      if(!effects.empty())
	search.insert(search.end(), effects.begin(), effects.end());
      std::list<EUROPA::TokenId>::iterator sToken = search.begin();
      while(sToken != search.end())
	{
	  if(m_assembly.is_goal((*sToken)))
	    {
	      return *sToken;
	    }
	  else if((*sToken)->isActive())
	    {
	      merged = (*sToken)->getMergedTokens();
	      if(!merged.empty())
		search.insert(search.end(), merged.begin(), merged.end());
	    }
	  slaves = (*sToken)->slaves();
	  if(!slaves.empty())
	    search.insert(search.end(), slaves.begin(), slaves.end());
	  sToken++;
	}
    }
  return EUROPA::Id<EUROPA::Token>::noId();
}

EUROPA::TokenSet CurrentState::getAllTokens(const EUROPA::TokenId& token)
{
  EUROPA::TokenSet merged;
  if(token.isId())
    {
      ///Gets all of the tokens merged with @token
      if(token->isActive())
	{
	  merged = token->getMergedTokens();
	  merged.insert(token);
	}
      else
	{
	  merged = token->getActiveToken()->getMergedTokens();
	  merged.insert(token->getActiveToken());
	}
    }
  return merged;
}


/*
 * class TREX::europa::details::CurrentState::DecisionPoint
 */

// statics

bool CurrentState::DecisionPoint::customStaticMatch(EUROPA::EntityId const &entity) {
  return CurrentStateId::convertable(entity);
}

// structors

CurrentState::DecisionPoint::DecisionPoint(EUROPA::DbClientId const &client,
					   CurrentStateId const &timeline,
					   EUROPA::TiXmlElement const &config,
					   EUROPA::LabelStr const &explanation)
  :EUROPA::SOLVERS::DecisionPoint(client, timeline->getKey(), explanation),
   m_target(timeline) {}

// observers

std::string CurrentState::DecisionPoint::toString() const {
  return toShortString();
}

std::string CurrentState::DecisionPoint::toShortString() const {
  std::ostringstream oss;

  oss<<"trex_";
  if( m_choices.none() )
    oss<<"EMPTY";
  else {
    switch( m_idx ) {
    case EXTEND_CURRENT:
      oss<<"EXTEND["<<m_target->current()->toString()<<']';
      break;
    case START_NEXT:
      oss<<"START["<<(*m_tok)->toString()<<']';
      break;
    case CREATE_DEFAULT:
      oss<<"DEFAULT["<<m_target->default_pred().toString()<<']';
      break;
    case CREATE_OTHER:
      oss<<"CREATE["<<m_next_pred->toString()<<']';
      break;
    default:
      oss<<"ERROR"; // I should never be here !!!!
    }
  }
  oss<<'('<<m_target->timeline()->toString()<<'['<<m_target->now()<<"])";
  return oss.str();
}

bool CurrentState::DecisionPoint::hasNext() const {
  return m_idx<m_choices.size();
}

bool CurrentState::DecisionPoint::canUndo() const {
  return EUROPA::SOLVERS::DecisionPoint::canUndo();
}

// modifiers

void CurrentState::DecisionPoint::handleInitialize() {
  EUROPA::TokenId cur = m_target->current(), active;
  EUROPA::eint date = m_target->now();
  
  debugMsg("trex:current", "Determining state choices for "<<m_target->timeline()->getName().toString()<<" at tick "<<date);
  m_choices.reset();
  debugMsg("trex:current", "Adding SET_DEFAULT by default");
  m_choices.set(CREATE_DEFAULT);
  
  if( cur.isId() ) {
    active = cur->getActiveToken();
    if( active.isNoId() )
      active = cur;
    EUROPA::IntervalIntDomain const &end_cur = active->end()->lastDomain();
    
    debugMsg("trex:current", "Checking if last observation can end after "<<date<<" (end="<<end_cur.toString()<<").");
    if( end_cur.getLowerBound()<=date ) {
      bool long_enough = ( end_cur.getUpperBound()>date );
      m_choices.set(EXTEND_CURRENT, long_enough);
      if( long_enough ) {
	debugMsg("trex:current", "Adding EXTEND_CURRENT as possible choice");
      } else {
	debugMsg("trex:current", "Cannot EXTEND_CURRENT for this timeline");
      }
    } else {
      m_choices.reset();
      m_choices.set(EXTEND_CURRENT);
      m_prev_idx= m_idx = EXTEND_CURRENT;
    
      debugMsg("trex:current", "EXTEND_CURRENT is the only choice");
      return;
    }
  }
  std::list<EUROPA::TokenId> const &sequence = m_target->timeline()->getTokenSequence();
  std::list<EUROPA::TokenId>::const_iterator i = sequence.begin();
  
  if( active.isId() ) {
    debugMsg("trex:current", "Look for successor of last observation.");
    for(; sequence.end()!=i && active!=*i; ++i);
    if( sequence.end()!=i )
      ++i;
    m_cand_to = m_cand_from = i;
    if( sequence.end()!=m_cand_from ) {
      EUROPA::TokenId tok = *m_cand_from;
      ++i;
      if( tok->start()->lastDomain().isMember(date) ) {
	m_cand_to = i;
	m_choices.set(START_NEXT);
	debugMsg("trex:current", "successor "<<tok->getPredicateName().toString()<<"("<<tok->getKey()<<") can be started (start="
		 <<tok->start()->lastDomain().toString()<<").");
      } else {
	debugMsg("trex:current", "No active successor starting over "<<date);
      }
    } else 
      debugMsg("trex:curent", "No active successor");
  } else {
    debugMsg("trex:current", "No active observation => find potential observation in the plan.");
    // skip tokens that necessarily start in the past 
    for( ; sequence.end()!=i && (*i)->start()->lastDomain().getUpperBound()<date; ++i);
    m_cand_from = i;
    for( ; sequence.end()!=i && (*i)->start()->lastDomain().isMember(date); ++i) {
      debugMsg("trex:current", "Could start "<<(*i)->getPredicateName().toString()<<"("<<(*i)->getKey()<<").");
    }
    m_cand_to = i;
    m_choices.set(START_NEXT, m_cand_from!=m_cand_to);
  }
  m_tok = m_cand_from;

  m_choices.set(CREATE_OTHER, !m_target->m_pred_names.empty());
  m_next_pred = m_target->m_pred_names.begin();
  
  if( m_choices.none() )
    debugMsg("trex:always", "No choices for current state flaw of timeline "<<m_target->timeline()->getName().toString());
  
  // Initialize the decision index
  m_idx = 0;
  if( !m_choices[m_idx] )
    advance();
  m_prev_idx = m_idx;
}

void CurrentState::DecisionPoint::handleExecute() {
  EUROPA::TokenId tok;

  m_prev_idx = m_idx;
  switch(m_idx) {
  case EXTEND_CURRENT:
    debugMsg("trex:current", "Execute decision EXTEND_END");
    m_target->push_end();
    break;
  case START_NEXT:
    debugMsg("trex:current", "Execute decision START_NEXT "<<(*m_tok)->getPredicateName().toString()<<".start="<<(*m_tok)->start()->lastDomain().toString());
    tok = m_target->new_obs((*m_tok)->getPredicateName().toString(), false);      
    details::restrict_attributes(tok, *m_tok);
    m_client->merge(tok, *m_tok);
    break;
  case CREATE_DEFAULT:
    debugMsg("trex:current", "Execute decision CREATE "<<m_target->default_pred().toString());
    tok = m_target->new_obs(details::predicate_name(m_target->timeline(), m_target->default_pred()),
			    false);
    break;
  case CREATE_OTHER:
    debugMsg("trex:current", "Execute decision CREATE "<<m_next_pred->toString());
    tok = m_target->new_obs(details::predicate_name(m_target->timeline(), *m_next_pred),
			    false);
    break;
  default:
    throw EuropaException("Unknown synchronization decision for timeline "+
			  m_target->timeline()->toString());
  }
}

void CurrentState::DecisionPoint::handleUndo() {
  switch(m_idx) {
  case EXTEND_CURRENT:
    m_target->relax_end();
    advance();
    break;
  case START_NEXT:
    m_target->relax_token();
    ++m_tok;
    if( m_cand_to==m_tok )
      advance();
    break;
  case CREATE_DEFAULT:
    m_target->relax_token();
    advance();
    break;
  case CREATE_OTHER:
    m_target->relax_token();
    ++m_next_pred;
    if( m_target->m_pred_names.end()==m_next_pred )
      advance();
    break;
  default:
    throw EuropaException("Unknown Synchronization decision for timeline "+
			  m_target->timeline()->toString());
  }
}

void CurrentState::DecisionPoint::advance() {
  do {
    ++m_idx;
  } while( m_idx<m_choices.size() && !m_choices[m_idx] );
}

/*
 * class TREX::europa::details::UpdateMatchFinder
 */

void UpdateMatchFinder::getMatches(EUROPA::SOLVERS::MatchingEngineId const &engine,
				   EUROPA::EntityId const &entity,
				   std::vector<EUROPA::SOLVERS::MatchingRuleId> &result) {
  return engine->getMatches(CurrentStateId(entity), result);
}

/*
 * class TREX::europa::details::TimePoint
 */

EUROPA::IntervalIntDomain TimePoint::future(EUROPA::eint now) {
  return 
    EUROPA::IntervalIntDomain(now+1, 
			      std::numeric_limits<EUROPA::eint>::infinity());
}

// structors 
 
TimePoint::TimePoint(EUROPA::TokenId const &tok,
		     EUROPA::eint now,
		     EUROPA::LabelStr const &name) 
  :EUROPA::Variable<EUROPA::IntervalIntDomain>
   (tok->getPlanDatabase()->getConstraintEngine(), 
    future(now), true, true, name),
   m_tok(tok) {
  debugMsg("trex:TimePoint", "Create "<<name.toString()<<" for token "<<m_tok->getPredicateName()
	   <<"("<<m_tok->getKey()<<")");
  if( m_tok.isId() ) {
    debugMsg("trex:TimePoint", "make local to token "<<m_tok->getPredicateName()
	   <<"("<<m_tok->getKey()<<")");
    m_tok->addLocalVariable(getId());
    create_constraint();
  }
}

TimePoint::~TimePoint() {
  // if( m_tok.isId() )
  //   m_tok->removeLocalVariable(getId());
}
 
// modifiers 

bool TimePoint::set_date(EUROPA::eint now) {
  EUROPA::Domain &dom = *m_derivedDomain;

  dom.intersect(future(now));
  if( derivedDomain().isEmpty() )
    return false;
  touch();
  return true;
}

void TimePoint::relax_date() {
  m_derivedDomain->relax(baseDomain()); 
}

bool TimePoint::commit_date(EUROPA::eint now) {

  if( lastDomain().getUpperBound()<=now )
    return false;
  else {
    restrictBaseDomain(future(now));
    return true;
  }
}

bool TimePoint::commit_end(EUROPA::eint now) {
  if( lastDomain().getUpperBound()<now ||
      lastDomain().getLowerBound()>now )
    return false;
  else {
    restrictBaseDomain(EUROPA::IntervalIntDomain(now));
    return true;
  }
}

void TimePoint::setToken(EUROPA::TokenId tok) {
  if( m_tok!=tok ) {
    std::swap(tok, m_tok);
    if( m_tok.isId() ) {
      m_tok->addLocalVariable(getId());
      create_constraint();
    }
  }
}

void TimePoint::create_constraint() {
  if( m_tok.isId() ) {
    debugMsg("trex:TimePoint", "Adding "<<getName().toString()<<'('
	     <<getKey()<<")=="<<m_tok->getPredicateName()<<'('
	     <<m_tok->getKey()<<")");
    std::vector<EUROPA::ConstrainedVariableId> scope(2);
    scope[0] = getId();
    scope[1] = m_tok->end();
    m_constraint = m_tok->getPlanDatabase()->getClient()->createConstraint("concurrent", scope, "trex inertial value");
    // m_tok->addStandardConstraint(m_constraint);    
    // m_constraint->undoDeactivation();
  } else {
    debugMsg("trex:TimePoint", getName().toString()
             <<'('<<getKey()<<") is not associated to a token");
  }
}
