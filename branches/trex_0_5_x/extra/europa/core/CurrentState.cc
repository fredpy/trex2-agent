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

// include plasma header as system files in order to disable warnings
# define TREX_PP_SYSTEM_FILE <PLASMA/TokenVariable.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/PlanDatabaseWriter.hh>
# include <trex/europa/bits/system_header.hh>

#include <trex/utils/platform/chrono.hh>
#include <boost/unordered_map.hpp>

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
    m_frontier->pending_discard();
    m_frontier = EUROPA::ConstrainedVariableId::noId();
  }
  else if( m_prev_obs==token ) {
    m_prev_obs = EUROPA::TokenId::noId();
    m_prev_frontier->pending_discard();
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

namespace {
  std::string new_frontier() {
    static size_t count = 0;
    std::ostringstream oss;
    oss<<"__trex_frontier_"<<(count++);
    return oss.str();
  }
}


void CurrentState::new_token(EUROPA::TokenId const &token) {
  m_prev_obs = m_last_obs;
  if( m_prev_frontier.isId() )
    m_prev_frontier->pending_discard();
  m_prev_frontier = m_frontier;
  m_last_obs = token;
  m_last_obs->incRefCount();
  m_frontier = (new TimePoint(m_last_obs, now(), new_frontier()))->getId();
}

void CurrentState::relax_token() {
  m_client->deleteToken(m_last_obs);
  m_last_obs = m_prev_obs;
  if( m_frontier.isId() ) {
    m_frontier->pending_discard();
  }
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
      m_prev_obs->end()->restrictBaseDomain(m_prev_frontier->lastDomain());
      m_assembly.terminate(m_prev_obs);
      m_prev_obs = EUROPA::TokenId::noId();
      if( m_prev_frontier.isId() )
        m_prev_frontier->pending_discard();
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

  if( !m_assembly.constraint_engine()->isPropagating() ) {
    if( m_assembly.constraint_engine()->propagate() )
      return true;
    debugMsg("trex:always", "["<<now()
             <<"] Inconsitency detected after applying observation "
             <<timeline()->toString()<<"."
             <<m_last_obs->getUnqualifiedPredicateName().toString()
             <<'('<<m_last_obs->getKey()<<").");
    return false;
  } else
    return true;
}

// manipulators

void CurrentState::do_dispatch(EUROPA::eint lb, EUROPA::eint ub) {
   static bool bfs(true), dist(false);

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
	     <<"] as a goal (start="<<(*i)->start()->lastDomain().toString()<<")");
    typedef Assembly::thread_clock thread_clock;
    
      // Used to find the percision of the clock
	  //std::cout << (double) CHRONO::high_resolution_clock::period::num
      //                / CHRONO::high_resolution_clock::period::den;
     typedef thread_clock::duration thread_duration;
    if(bfs)
    {
        thread_duration duration;
        thread_clock::time_point start = thread_clock::now();
        EUROPA::TokenId nowGoal = getGoal(*i, lb, ub);
        if(nowGoal.isId())
        {
            duration=thread_clock::now()-start;
            debugMsg("trex:dispatch", "Token "<<(*i)->getUnqualifiedPredicateName().toString()<<"("<<(*i)->getKey()<<") schedulled for dispatch."
                     "\n\treason: linked to goal "<<nowGoal->getUnqualifiedPredicateName().toString()
                     <<'('<<nowGoal->getKey()<<").");
            m_assembly.dispatch(timeline(),*i);
        } else {
            duration=thread_clock::now()-start;
          debugMsg("trex:dispatch", "Holding on "<<(*i)->getUnqualifiedPredicateName().toString()<<"("<<(*i)->getKey()<<") should not be dispatched yet.")
        }
        if(time_values.find(now())==time_values.end())
        {
            time_values[now()]=duration.count();
        } else {
            time_values[now()]+=duration.count();
        }
        //std::cout<<now()<<": "<<time_values[now()]<<" | "<<(*i)->getUnqualifiedPredicateName().toString()
	    // <<'('<<(*i)->getKey()<<")"<<std::endl;
    }
    /** @brief Dispatching function with the help of listener_proxy
    *   Using this code will allow for the token to be tested if it is a subgoal or goal itself
    *   for dispatching. Must have a goal<-effect<-action->condition->token relationship in
    *   the model to take advantage of this dispatching method.
    */
    if(dist)
    {
        thread_duration duration;
        thread_clock::time_point start = thread_clock::now();
        if(dispatch_token(*i,lb,ub))
        {
            duration=thread_clock::now()-start;
            m_assembly.dispatch(timeline(),(*i));
        } else {
            duration=thread_clock::now()-start;
        }
        if(m_assembly.time_values.find(now())==m_assembly.time_values.end())
        {
            m_assembly.time_values[now()]=duration.count();
        } else {
            m_assembly.time_values[now()]+=duration.count();
        }
        //std::cout<<"Search: "<<now()<<": "<<m_assembly.time_values[now()]<<std::endl;
    }

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

//
//    if(bfs)
//    {
//      if(lb==m_assembly.final_tick())
//      {
//        std::ofstream time_file;
//        time_file.open("time_file.csv", std::ofstream::app | std::ofstream::out | std::ofstream::ate);
//        for(EUROPA::eint i=0; i<=m_assembly.final_tick(); i++)
//        {
//            if(i>0)
//                time_file<<", ";
//            if(time_values.find(i)!=time_values.end())
//                time_file<<time_values[i];
//            else
//                time_file<<"0";
//        }
//        time_file<<std::endl;
//        time_file.close();
//      }
//    }
//
//    if(dist)
//    {
//      if(lb==m_assembly.final_tick())
//      {
//        std::ofstream time_file;
//        time_file.open("time_file.csv", std::ofstream::app | std::ofstream::out | std::ofstream::ate);
//        for(EUROPA::eint i=0; i<=m_assembly.final_tick(); i++)
//        {
//            if(i>0)
//                time_file<<", ";
//            if(m_assembly.time_values.find(i)!=m_assembly.time_values.end())
//                time_file<<m_assembly.time_values[i];
//            else
//                time_file<<"0";
//        }
//        time_file<<std::endl;
//        time_file.close();
//      }
//    }
}

bool CurrentState::dispatch_token(const EUROPA::TokenId& token,EUROPA::eint lb, EUROPA::eint ub)
{
    if(m_assembly.is_subgoal(token) || m_assembly.is_action(token))
    {
        return true;
    }
    //Wait to dispatch the token until the last moment
    else if(token->start()->lastDomain().getUpperBound()<=ub)
    {
        return true;
    }
    return false;
}

EUROPA::TokenId  CurrentState::getGoal(const EUROPA::TokenId& token, EUROPA::eint lb, EUROPA::eint ub)
{
    if(token->start()->lastDomain().getUpperBound()<=ub || m_assembly.is_action(token))
    {
        debugMsg("trex:dispatch", token->getUnqualifiedPredicateName().toString()
                  <<'('<<token->getKey()<<") necessraly starts in ["<<lb<<", "
                  <<ub<<"] or before");
        return token;
    } else {
        EUROPA::TokenSet merged;
        EUROPA::TokenSet::iterator it, end;
        debugMsg("trex:dispatch", token->getUnqualifiedPredicateName().toString()
                 <<'('<<token->getKey()<<") can start after ["<<lb<<", "
               <<ub<<"] => checking if it is goal dependendent");

        ///Gets all of the tokens merged with @token
        merged = getAllTokens(token);
        ///Tests to see if any of the tokens are fact and if so @returns noId()
        for(it = merged.begin(), end = merged.end(); it!=end; it++)
        {
          if((*it)->isFact()) {
            debugMsg("trex:dispatch", token->getUnqualifiedPredicateName().toString()
                     <<'('<<token->getKey()<<") is already a fact : skipping");
            return EUROPA::Id<EUROPA::Token>::noId();
          }
        }

        EUROPA::TokenId goal = searchGoal(merged);
      if(goal.isId()) {
        debugMsg("trex:dispatch", token->getUnqualifiedPredicateName().toString()
                 <<'('<<token->getKey()<<") depends on a goal.");

        return goal;
      }
    }
    debugMsg("trex:dispatch", token->getUnqualifiedPredicateName().toString()
             <<'('<<token->getKey()<<") is not urgent.");

    return EUROPA::Id<EUROPA::Token>::noId();
}

EUROPA::TokenId CurrentState::searchGoal(const EUROPA::TokenSet& tokens)
{
    boost::unordered_map<long, bool> mark;
    std::list<EUROPA::TokenId> list;
    for(EUROPA::TokenSet::const_iterator it = tokens.begin(); it!=tokens.end(); ++it)
    {
        list.push_back(*it);
        mark[(*it)->getKey().asLong()]=true;
    }

    EUROPA::TokenSet effects, merged;
    EUROPA::TokenSet::iterator effToken, mergedIt;
    while(!list.empty())
    {
        EUROPA::TokenId token = list.front();
        list.pop_front();
        if(m_assembly.is_goal(token))
            return token;
        if(m_assembly.is_condition(token))
        {
            EUROPA::TokenId master = token->master();
            if(master.isId() && m_assembly.is_action(master))
            {
                effects = actionEffects(master);
                for(effToken = effects.begin(); effToken!=effects.end(); ++effToken)
                {
                    if(!mark[(*effToken)->getKey().asLong()])
                    {
                        merged = getAllTokens(*effToken);
                        for(mergedIt=merged.begin(); mergedIt!=merged.end(); ++mergedIt)
                        {
                            mark[(*mergedIt)->getKey().asLong()]=true;
                            list.push_back(*mergedIt);
                        }
                    }
                }
            }
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
        else if( token->isMerged() )
        {
          merged = token->getActiveToken()->getMergedTokens();
          merged.insert(token->getActiveToken());
        }
    }
    return merged;
}

EUROPA::TokenSet CurrentState::actionEffects(const EUROPA::TokenId& action)
{
    EUROPA::TokenSet slaves = action->slaves(), effects;
    for(EUROPA::TokenSet::iterator it = slaves.begin(), end = slaves.end();
        it!=end; it++)
    {
        if(m_assembly.is_effect((*it)))
            effects.insert(*it);
    }
    return effects;
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
   m_tok(tok), m_pending(false) {
  debugMsg("trex:synch:tp", "Create "<<name.toString()<<" for token "<<m_tok->getPredicateName().toString()
	   <<"("<<m_tok->getKey()<<")");
  // incRefCount();
  if( m_tok.isId() ) {
    debugMsg("trex:synch:tp", getKey()<<" made local to token "
             <<m_tok->getPredicateName().toString()
	   <<"("<<m_tok->getKey()<<")");
    // m_tok->addLocalVariable(getId());
    m_tok->addDependent(this);
    create_constraint();
  }
  debugMsg("trex:synch:tp", "TimePoint "<<getKey()<<"="<<baseDomain().toString()
           <<" CREATED.");
}

TimePoint::~TimePoint() {
  debugMsg("trex:synch:tp", "TimePoint "<<getKey()<<"="<<baseDomain().toString()
           <<" DESTROYED.");
  detach(false);
}

// modifiers

bool TimePoint::set_date(EUROPA::eint now) {
  EUROPA::Domain &dom = *m_derivedDomain;

  dom.intersect(future(now));
  if( lastDomain().isEmpty() ) {
      debugMsg("trex:synch:tp", "TimePoint "<<getKey()<<" emptied by ["<<now+1<<", +inf]");
    // touch();
    return false;
  }
  debugMsg("trex:synch:tp", "TimePoint "<<getKey()<<"*["<<now+1<<", +inf] = "
           <<lastDomain().toString());
  // touch();
  return true;
}

void TimePoint::relax_date() {
  m_derivedDomain->relax(baseDomain());
  debugMsg("trex:synch:tp", "TimePoint "<<getKey()<<" relaxed to "<<lastDomain().toString());
  // touch();
}

bool TimePoint::commit_date(EUROPA::eint now) {
  if( lastDomain().getUpperBound()<=now ) {
    debugMsg("trex:synch:tp", "TimePoint "<<getKey()<<"="<<lastDomain().toString()
             <<" cannot be committed to ["<<now+1<<", +inf]");
    return false;
  } else {
    restrictBaseDomain(future(now));
    // touch();

    debugMsg("trex:synch:tp", "TimePoint "<<getKey()<<"="<<lastDomain().toString()
             <<" after commit to ["<<now+1<<", +inf]");
    return true;
  }
}

bool TimePoint::commit_end(EUROPA::eint now) {
  if( lastDomain().getUpperBound()<now ||
      lastDomain().getLowerBound()>now ) {
    debugMsg("trex:synch:tp", "TimePoint "<<getKey()<<"="<<lastDomain().toString()
             <<" cannot be committed to {"<<now<<"}");
    return false;
  } else {
    restrictBaseDomain(EUROPA::IntervalIntDomain(now));
    // touch();
    debugMsg("trex:synch:tp", "TimePoint "<<getKey()<<"="<<lastDomain().toString()
             <<" after commit to {"<<now<<"}");
    return true;
  }
}

void TimePoint::setToken(EUROPA::TokenId tok) {
  if( m_tok!=tok ) {
    detach();
    m_tok = tok;
    if( m_tok.isId() ) {
      // m_tok->addLocalVariable(getId());
      m_tok->addDependent(this);
      create_constraint();
      debugMsg("trex:synch:tp", "TimePoint "<<getKey()<<" associated to token "
               <<tok->getPredicateName().toString()<<'('<<tok->getKey()<<')');
    }
  }
}

void TimePoint::create_constraint() {
  if( m_tok.isId() ) {
    debugMsg("trex:synch:tp", "Adding "<<getName().toString()<<'('
	     <<getKey()<<")=="<<m_tok->getPredicateName().toString()<<'('
	     <<m_tok->getKey()<<")");
    std::vector<EUROPA::ConstrainedVariableId> end_scope(2),
      dur_scope(3);
    EUROPA::DbClientId cli = m_tok->getPlanDatabase()->getClient();
    end_scope[0] = getId();
    end_scope[1] = m_tok->end();
    m_end_cstr = cli->createConstraint("concurrent", end_scope,
                                       "trex inertial value");
    dur_scope[0] = m_tok->start();
    dur_scope[1] = m_tok->duration();
    dur_scope[2] = getId();
    m_dur_cstr = cli->createConstraint("temporalDistance",
                                       dur_scope,
                                       "trex inertial value");
  } else {
    debugMsg("trex:synch:tp", getName().toString()
             <<'('<<getKey()<<") is not associated to a token");
  }
}

void TimePoint::detach(bool cstr) {
  if( m_tok.isId() ) {
    debugMsg("trex:synch:tp", "Detaching timepoint "<<getKey()<<" from token "
             <<m_tok->getPredicateName().toString()<<'('<<m_tok->getKey()<<')');
    if( m_dur_cstr.isId() ) {
      EUROPA::DbClientId cli = m_tok->getPlanDatabase()->getClient();
      if( cstr )
      cli->deleteConstraint(m_dur_cstr);

      m_dur_cstr = EUROPA::ConstraintId::noId();
      if( m_end_cstr.isId() ) {
        if( cstr )
          cli->deleteConstraint(m_end_cstr);
        m_end_cstr = EUROPA::ConstraintId::noId();
      }

    }
    // m_tok->removeLocalVariable(getId());
    if( cstr )
      m_tok->removeDependent(this);
    m_tok = EUROPA::TokenId::noId();
  }
}

void TimePoint::pending_discard() {
  m_pending = true;
  if( m_tok.isNoId() ) {
    debugMsg("trex:synch:tp", "Discard tp "<<getKey()
             <<" as it is no longer required.");
    // decRefCount();
  }
}


void TimePoint::notifyDiscarded(Entity const *entity) {
  if( m_tok.isId() ) {
    if( entity->getKey()==m_tok->getKey() ) {
      debugMsg("trex:synch:tp", "token "<<m_tok->getPredicateName().toString()
               <<'('<<m_tok->getKey()<<") discarded => detach it from tp "
               <<getKey());
      detach(false);
      if( m_pending )
        decRefCount();
    }
  }
}

void TimePoint::updateBase(EUROPA::IntervalIntDomain const &dom) {
  restrictBaseDomain(dom);
  touch();
  if( m_tok.isId() ) {
    m_tok->duration()->touch();
    m_tok->end()->touch();
  }
}


