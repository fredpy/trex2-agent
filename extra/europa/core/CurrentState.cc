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
  :m_assembly(assembly), m_timeline(timeline), m_id(this) {
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
    && current()->end()->baseDomain().getLowerBound() > now();
}

bool CurrentState::internal() const {
  return m_assembly.internal(EUROPA::ObjectId(m_timeline));
}

bool CurrentState::external() const {
  return m_assembly.external(EUROPA::ObjectId(m_timeline));
}

// modifiers

EUROPA::TokenId CurrentState::new_obs(EUROPA::DbClientId const &cli, std::string const &pred, 
				      bool insert) {
  EUROPA::IntervalIntDomain future(now()+1, std::numeric_limits<EUROPA::eint>::infinity());

  debugMsg("trex:state", "["<<now()<<"] Creating new observation "<<pred<<" on "<<timeline()->toString());
  m_prev_obs = m_last_obs;
  m_last_obs = cli->createToken(pred.c_str(), NULL, false, true);
  
  m_last_obs->start()->specify(now());
  m_last_obs->end()->restrictBaseDomain(future);
  m_last_obs->getObject()->specify(m_timeline->getKey());
  
  if( insert ) {
    debugMsg("trex:state", "["<<now()<<"] Creating new observation "<<pred<<" on "<<timeline()->toString());
    if( m_prev_obs.isId() )
      cli->constrain(m_timeline, m_prev_obs, m_last_obs);
  }
  return current();
}

void CurrentState::erased(EUROPA::TokenId const &token) {
  if( m_last_obs==token )
    m_last_obs = EUROPA::TokenId::noId();
  else if( m_prev_obs==token )
    m_prev_obs = EUROPA::TokenId::noId();
}

void CurrentState::apply_base(EUROPA::TokenId &merged) {
  EUROPA::TokenId active = merged->getActiveToken();
  std::vector<EUROPA::ConstrainedVariableId> const &actives = active->getVariables();
  std::vector<EUROPA::ConstrainedVariableId> const &mergeds = merged->getVariables();
  
  for(size_t v=1; v<actives.size(); ++v)
    actives[v]->restrictBaseDomain(mergeds[v]->lastDomain());
  merged = active;
}

void CurrentState::replaced(EUROPA::TokenId const &token) {
  EUROPA::eint t_start;

  if( m_last_obs==token ) {
    //    t_start = m_last_obs->start()->getSpecifiedValue();
    apply_base(m_last_obs);
    //m_last_obs->start()->specify(t_start);
  } else if( m_prev_obs==token ) {
    //t_start = m_prev_obs->start()->getSpecifiedValue();
    apply_base(m_last_obs);
    // m_prev_obs->start()->specify(t_start);
  }
}


void CurrentState::new_token(EUROPA::TokenId const &token) {
  m_prev_obs = m_last_obs;
  m_last_obs = token;
}

void CurrentState::relax_token() {
  m_last_obs->discard();
  m_last_obs = m_prev_obs;
  m_prev_obs = EUROPA::TokenId::noId();
}

void CurrentState::push_end(EUROPA::DbClientId const &cli) {
  std::vector<EUROPA::ConstrainedVariableId> lt_scope(2);

  lt_scope[0] = m_assembly.clock();
  lt_scope[1] = m_last_obs->end();
  m_constraint = cli->createConstraint("lt", lt_scope, "Inertial value asumption");
}

void CurrentState::relax_end(EUROPA::DbClientId const &cli) {
  cli->deleteConstraint(m_constraint);
  m_constraint = EUROPA::ConstraintId::noId();
}

void CurrentState::commit() {  
  debugMsg("trex:synch", "["<<now()<<"] commit "<<m_last_obs->toString()<<" on "<<timeline()->toString());
  if( m_prev_obs.isId() ) {
    debugMsg("trex:synch", "Terminating "<<m_prev_obs->toString()<<'('<<m_prev_obs->getKey()<<")");
    EUROPA::eint start = m_prev_obs->start()->getSpecifiedValue();

//    {
//      scoped_split prev(m_prev_obs);
//      prev->end()->restrictBaseDomain(EUROPA::IntervalIntDomain(now()));
//      prev->duration()->restrictBaseDomain(EUROPA::IntervalIntDomain(now()-start));
//    }
    restrict_base(m_prev_obs, m_prev_obs->end(), EUROPA::IntervalIntDomain(now()));
    restrict_base(m_prev_obs, m_prev_obs->duration(), EUROPA::IntervalIntDomain(now()-start));

    m_assembly.terminate(m_prev_obs);
    m_prev_obs = EUROPA::TokenId::noId();
    restrict_base(m_last_obs, m_last_obs->start(), EUROPA::IntervalIntDomain(now()));
    m_assembly.notify(*this);
    if( m_constraint.isId() ) {
      m_assembly.plan_db()->getClient()->deleteConstraint(m_constraint);
      m_constraint = EUROPA::ConstraintId::noId();
    }
  } else {
    EUROPA::eint start = m_last_obs->start()->getSpecifiedValue();
    if( now()==start ) {
      restrict_base(m_last_obs, m_last_obs->start(), EUROPA::IntervalIntDomain(now()));
      m_assembly.notify(*this);      
    } else {
      EUROPA::IntervalIntDomain future(now()+1, std::numeric_limits<EUROPA::eint>::infinity()),
        dur(now()+1-start, std::numeric_limits<EUROPA::eint>::infinity());
    
//      {
//        scoped_split cur(m_last_obs);  
//        
//        cur->end()->restrictBaseDomain(future);
//        cur->duration()->restrictBaseDomain(dur);
//        m_assembly.constraint_engine()->propagate();
//        debugMsg("trex:synch", "Plan during commit["<<timeline()->toString()<<"]:\n"
//                 <<EUROPA::PlanDatabaseWriter::toString(m_assembly.plan_db()));
//      }
      restrict_base(m_last_obs, m_last_obs->end(), future);
      restrict_base(m_last_obs, m_last_obs->duration(), dur);
      m_assembly.constraint_engine()->propagate();
    }
    if( m_constraint.isId() ) {
      m_assembly.plan_db()->getClient()->deleteConstraint(m_constraint);
      m_constraint = EUROPA::ConstraintId::noId();
    }
  }
  // if( m_last_obs->canBeCommitted() )
  //   m_last_obs->commit();
  debugMsg("trex:synch", "["<<now()<<"] end commit on "<<timeline()->toString());
}

// manipulators

void CurrentState::do_dispatch(EUROPA::eint lb, EUROPA::eint ub) {
  std::list<EUROPA::TokenId>::const_iterator 
    i = timeline()->getTokenSequence().begin(), 
    endi = timeline()->getTokenSequence().end();
  // skip the past tokens
  for( ; endi!=i && (*i)->start()->lastDomain().getUpperBound()<lb; ++i);

  for( ; endi!=i && (*i)->end()->lastDomain().getLowerBound()<=ub; ++i)
    if( !m_assembly.dispatch(timeline(), *i) )
      break;
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
  

  m_choices.reset();
  m_choices.set(CREATE_DEFAULT);
  

  if( cur.isId() ) {
    active = cur->getActiveToken();
    if( active.isNoId() )
      active = cur;
    EUROPA::IntervalIntDomain const &end = active->end()->lastDomain();
    
    if( end.getLowerBound() <= date ) {
      m_choices.set(EXTEND_CURRENT, end.getUpperBound() > date);
    } else {
      // Already decided : extend_current
      m_choices.reset();
      m_idx = m_choices.size();
      return;
    }
  }
  // try to find the potential candidates
  std::list<EUROPA::TokenId> const &sequence = m_target->timeline()->getTokenSequence();
  std::list<EUROPA::TokenId>::const_iterator i=sequence.begin();
  if( active.isId() ) {
    // Last observation is active => find it 
    for(; sequence.end()!=i && active!=*i; ++i);
    // Then make the next token in the sequence the candidate
    if( sequence.end()!=i )
      ++i;
    m_cand_from = i;
    if( sequence.end()!=i )
      ++i;
    m_cand_to = i;
  } else {
    // No last observation active => search for tokens that can start now
    for(; sequence.end()!=i && (*i)->end()->lastDomain().getUpperBound()<=date; ++i);
    m_cand_from = i;
    for(; sequence.end()!=i && (*i)->start()->lastDomain().getUpperBound()<=date; ++i);
    m_cand_to = i;
  }
  // <m_cand_from, m_cand_to> reflects all the tokens in the sequence that can potentially overlap now
  m_choices.set(START_NEXT, m_cand_from!=m_cand_to);
  m_tok = m_cand_from;
  
  m_choices.set(CREATE_OTHER, !m_target->m_pred_names.empty());
  m_next_pred = m_target->m_pred_names.begin();
    
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
    m_target->push_end(m_client);
    break;
  case START_NEXT:
    tok = m_target->new_obs(m_client, (*m_tok)->getPredicateName().toString(), false);
    m_client->merge(tok, *m_tok);
    break;
  case CREATE_DEFAULT:
    tok = m_target->new_obs(m_client, 
			    details::predicate_name(m_target->timeline(), m_target->default_pred()),
			    true);
    break;
  case CREATE_OTHER:
    tok = m_target->new_obs(m_client, 
			    details::predicate_name(m_target->timeline(), *m_next_pred),
			    true);
    break;
  default:
    throw EuropaException("Unknown synchronization decision for timeline "+
			  m_target->timeline()->toString());
  }
}

void CurrentState::DecisionPoint::handleUndo() {
  switch(m_idx) {
  case EXTEND_CURRENT:
    m_target->relax_end(m_client);
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

