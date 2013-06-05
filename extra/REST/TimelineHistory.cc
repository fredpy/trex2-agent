/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2013, MBARI.
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
#include "TimelineHistory.hh"
#include "REST_reactor.hh"
#include "REST_service.hh"

namespace bp=boost::property_tree;
using namespace TREX::transaction;


using namespace TREX::REST;


/*
 * TREX::REST::TimelineHistory
 */

TimelineHistory::TimelineHistory(REST_reactor &creator)
:graph::timelines_listener(creator.get_graph()), m_fancy(true), m_reactor(creator),
 m_strand(creator.manager().service()) {
   boost::filesystem::path p = m_reactor.file_name("timelines"+helpers::db_manager::db_ext);
   m_db.initialize(p.string());
   
   // complete my initialization to receive the timleines crrated before I was created
   graph::timelines_listener::initialize();
}

TimelineHistory::~TimelineHistory() {}

// manipulators

bp::ptree TimelineHistory::get_goal(goal_id g) const {
  bp::ptree tmp;
  tmp.put("id", g);
  tmp.put("href", "/rest/goal/"+tmp.get<std::string>("id"));
  tmp.put_child("Goal", get_token(g));
  return tmp;
}


bp::ptree TimelineHistory::get_token(goal_id const &tok) const {
  return m_reactor.getGraph().export_goal(tok).get_child("Goal");
}

TICK TimelineHistory::get_date(std::string const &date) {
  return m_reactor.timeToTick(utils::string_cast<REST_reactor::date_type>(date));
}


// TREX updates callbacks

void TimelineHistory::new_obs(Observation const &obs, TICK cur) {
  goal_id tok(new Goal(obs, cur));
  m_strand.post(boost::bind(&TimelineHistory::add_obs_sync, this, tok, cur));
}

void TimelineHistory::update_tick(TICK cur) {
  m_strand.post(boost::bind(&TimelineHistory::ext_obs_sync, this, cur));
}


void TimelineHistory::declared(details::timeline const &timeline) {
  m_strand.post(boost::bind(&TimelineHistory::add_tl_sync, this, boost::ref(timeline)));
  m_reactor.use(timeline.name());
}

// REST callbacks

void TimelineHistory::list_timelines(std::ostream &out,
                                     std::set<std::string> const &select, bool hidden) {
  // I build the json by hand
  out<<"{ \"timelines\": [";
  
  boost::function<size_t ()> fn(boost::bind(&TimelineHistory::list_tl_sync, this, boost::ref(out), boost::ref(select), hidden));
  
  size_t count = utils::strand_run(m_strand, fn);
  
  if( count>0 )
    out.put(' ');
  out<<"] }";
}

void TimelineHistory::get_tokens(std::string const &timeline,
                                 IntegerDomain::bound &lo,
                                 IntegerDomain::bound const &hi,
                                 std::ostream &dest, bool first,
                                 size_t max) {
  utils::Symbol tl(timeline);
  
  boost::function<size_t ()>
  fn(boost::bind(&TimelineHistory::get_tok_sync, this,
                 tl, boost::ref(lo), hi, boost::ref(dest),
                 first, max));
  size_t count = utils::strand_run(m_strand, fn);
  if( count==0 && !first ) {
    dest.put('\n');
  } 
}

bool TimelineHistory::exists(std::string const &name) {
  utils::Symbol tl(name);
  boost::function<bool ()> fn(boost::bind(&TimelineHistory::exists_sync,
                                          this, tl));
  return utils::strand_run(m_strand, fn);
}

bp::ptree TimelineHistory::goals() {
  boost::function<bp::ptree ()> fn(boost::bind(&TimelineHistory::goals_sync, this));
  return utils::strand_run(m_strand, fn);
}

goal_id TimelineHistory::add_goal(std::string const &file) {
  bp::ptree data;
  try {
    std::ifstream in(file.c_str());
    utils::read_json(in, data);
  } catch(std::exception const &e) {
    m_reactor.syslog(utils::log::warn)<<"Failed to parse file \""<<file<<"\" as json:\n"<<e.what();
    throw std::runtime_error(std::string("error while parsing goal: ")+e.what());
  } catch(...) {
    m_reactor.syslog(utils::log::warn)<<"Failed to parse file \""<<file<<"\" as json with unknown error";
    throw std::runtime_error("Unknown error while parsing goal.");
  }
  if( data.empty() )
    throw std::runtime_error("goal json description is empty.");
  bp::ptree::value_type g_desc("goal", data);
  
  goal_id g = m_reactor.parse_goal(g_desc);
  
  if( !m_reactor.isExternal(g->object()) )
    throw std::runtime_error("Goal associated to unknown timeline \""+g->object().str()+"\"");
  
  m_reactor.postGoal(g);
  
  m_strand.dispatch(boost::bind(&TimelineHistory::add_goal_sync, this, g));
  
  return g;
}


goal_id TimelineHistory::get_goal(std::string const &id) {
  boost::function<goal_id ()> fn(boost::bind(&TimelineHistory::get_goal_sync, this, id));
  return utils::strand_run(m_strand, fn);
}

bool TimelineHistory::delete_goal(std::string const &id) {
  boost::function<goal_id ()> fn(boost::bind(&TimelineHistory::del_goal_sync, this, id));
  goal_id g = utils::strand_run(m_strand, fn);
  
  if( g )
    return m_reactor.postRecall(g); // Not sure if postRecall is thread safe ... 
  else
    return false;
}

// internal asynchronous calls : all these calls should be stranded

void TimelineHistory::add_tl_sync(details::timeline const &tl) {
  // insert the new timeline in my set
  helpers::timeline_wrap *entry = new helpers::timeline_wrap(tl);
  if( m_timelines.insert(entry).second ) {
    m_db.add_timeline(tl.name().str());
  } else
    delete entry;
}

void TimelineHistory::add_obs_sync(goal_id tok, TICK date) {
  helpers::rest_tl_set::iterator pos = m_timelines.find(tok->object());
  if( m_timelines.end()!=pos ) {    
    goal_id prev;
    TICK start;
    
    // Set in memory the new observation
    boost::tie(start, prev) = (*pos)->new_obs(date, tok);
    if( prev ) {
      // If there was a former observation then store it in the database
      // Do the export in json so the data is already formatted for the services
      std::ostringstream oss;
      helpers::json_stream json(oss);
      prev->restrictEnd(IntegerDomain(date));
      utils::write_json(json, get_token(prev), fancy());
      m_db.add_token(start, date, (*pos)->name().str(), oss.str());
    }
  } else
    m_reactor.syslog(utils::log::warn)<<"Received an observation on "<<tok->object()
    <<" which is not declared yet !!!";
}

void TimelineHistory::ext_obs_sync(TICK date) {
  m_cur = date;
  IntegerDomain future(date+1, IntegerDomain::plus_inf);
  
  for(helpers::rest_tl_set::iterator i=m_timelines.begin();
      m_timelines.end()!=i; ++i)
    if( (*i)->has_observation() )
      (*i)->obs()->restrictEnd(future);
}


size_t TimelineHistory::list_tl_sync(std::ostream &out, std::set<std::string> const &select, bool hidden) {
  size_t count =0;
  
  for(helpers::rest_tl_set::const_iterator i=m_timelines.begin(); m_timelines.end()!=i;
      ++i) {
    bool valid;
    if( select.empty() ) {      
      valid = hidden || 0!=(*i)->name().str().compare(0,1, "_");
    } else
      valid = select.end()!=select.find((*i)->name().str());
    
    if( valid ) {
      if( count>0 )
        out.put(',');
      ++count;
    
      TICK t_l = (*i)->latency(), t_pi = (*i)->look_ahead();
        
      out<<"\n  { \"name\": \""<<(*i)->name()<<"\","
      // href is hard coded .... I dshould be able to do better but will
      // do for now
         <<"\n    \"href\": \"/rest/timeline/"<<(*i)->name()<<"\","
         <<"\n    \"alive\": \""<<(*i)->alive()<<"\","
         <<"\n    \"accept_goals\": \""<<(*i)->accept_goals()<<"\","
         <<"\n    \"latency\": { \"ticks\": \""<<t_l<<"\", \"duration\": \""
         <<m_reactor.duration_str(t_l)<<"\" },"
         <<"\n    \"look_ahead\": { \"ticks\": \""<<t_pi<<"\", \"duration\": \""
         <<m_reactor.duration_str(t_pi)<<"\" },"
         <<"\n    \"publish_plan\": \""<<(*i)->publish_plan()<<"\","
         <<"\n    \"total_obs\": "<<(*i)->count()<<",";

      typedef utils::chrono_posix_convert<TeleoReactor::duration_type> convert;
      convert::posix_duration period = convert::to_posix(m_reactor.tickDuration());
      long double factor = 0.0;
      if( (*i)->count()>0 ) {
        TICK n_ticks = 1+m_cur-(*i)->initial();
        factor = n_ticks;
        factor /= (*i)->count();
        period *= n_ticks;
        period /= (*i)->count();
      } else
        period *= 0;
    
      out<<"\n    \"obs_period\": { \"ticks\": \""<<factor<<"\", "
         <<"\"duration\": \""<<period<<"\" }\n  }";
    }
  }
  
  return count;
}

bool TimelineHistory::exists_sync(TREX::utils::Symbol name) {
  return m_timelines.end()!=m_timelines.find(name);
}


size_t TimelineHistory::get_tok_sync(TREX::utils::Symbol tl,
                                     IntegerDomain::bound &lo,
                                     IntegerDomain::bound hi,
                                     std::ostream &out,
                                     bool first, size_t max) {
  size_t ret = 0;
  helpers::rest_tl_set::const_iterator pos = m_timelines.find(tl);
  if( m_timelines.end()!=pos ) {
    if( !first )
      out.put(',');
    if( (*pos)->has_observation() ) {
      IntegerDomain::bound date = (*pos)->obs_date();
      if( date>=lo ) {
        // access the database
        ret = m_db.get_tokens(tl.str(), lo, hi, out, max);
        if( ret==max )
          return ret;
        else if( ret>0 )
          first = false;
      }
      // From there on the only things remaining are in memory
      // these ones need to be displayed at once
      if( date<=hi ) {
        if( !first )
          out.put(',');
        else
          first = false;
        utils::write_json(out, get_token((*pos)->obs()), fancy());
        lo = IntegerDomain::plus_inf;
        ret += 1;
      }
      // TODO : repeat this for the planned tokens
    }
  }
  return ret;
}


bp::ptree TimelineHistory::goals_sync() {
  bp::ptree ret;
  
  for(goal_map::const_iterator i=m_goals.begin(); m_goals.end()!=i; ++i)
    ret.push_back(bp::ptree::value_type("", get_goal(i->second)));
  return ret;
}

void TimelineHistory::add_goal_sync(goal_id g) {
  std::ostringstream oss;
  oss<<g;
  
  m_goals[oss.str()] = g;
}

goal_id TimelineHistory::get_goal_sync(std::string id) {
  goal_map::iterator i = m_goals.find(id);
  if( m_goals.end()==i )
    return goal_id();
  else
    return i->second;
}

goal_id TimelineHistory::del_goal_sync(std::string id) {
  goal_map::iterator i = m_goals.find(id);
  goal_id result;
  if( m_goals.end()!=i ) {
    result = i->second;
    m_goals.erase(i);
  }
  return result;
}









