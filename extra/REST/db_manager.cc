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
#include "db_manager.hh"
#include "dbo_cfg.hh"

#include <Wt/Dbo/Dbo>

namespace dbo = Wt::Dbo;

/*
 * databasxe tables and relations
 */

namespace  {
  typedef dbo::backend::DBO_BACKEND db_type;
  
  class db_token;
  
  typedef dbo::collection< dbo::ptr<db_token> > db_tokens;
  
  class db_timeline {
  public:
    db_timeline() {}
    db_timeline(std::string const &n):name(n) {}
    
    std::string name;
    db_tokens   observations;
    
    template<class Action>
    void persist(Action &a) {
      dbo::id(a, name, "name");
      dbo::hasMany(a, observations, dbo::ManyToOne, "timeline");
    }
  }; // db_timeline
  
}

namespace Wt {
  namespace Dbo {
    
    template<>
    struct dbo_traits<db_timeline>
    :public dbo_default_traits {
      typedef std::string IdType;
      
      static IdType invalidId() {
        return std::string();
      }
      
      static const char *surrogateIdField() { return 0; }
    }; // Wt::Dbo::dbo_traits<db_timeline>
    
  }
}

namespace {
  
  class db_token {
  public:
    TREX::transaction::TICK start, end;
    dbo::ptr<db_timeline>   timeline;
    std::string             json;
    
    template<class Action>
    void persist(Action &a) {
      dbo::field(a, start, "start");
      dbo::field(a, end, "end");
      dbo::belongsTo(a, timeline, dbo::NotNull);
      dbo::field(a, json, "json");
    }
    
  }; // db_token
  
}

using namespace TREX::REST::helpers;


/*
 * TREX::REST::helpers::db_manager
 */

std::string const db_manager::db_ext(DBO_EXTENSION);

db_manager::db_manager() {}

db_manager::~db_manager() {}

void db_manager::initialize(std::string const &file_name) {
  m_db.reset(new db_type(file_name));
  m_session.setConnection(*m_db);
  
  m_session.mapClass<db_timeline>("timeline");
  m_session.mapClass<db_token>("token");
  m_session.createTables();
}

dbo::Session &db_manager::session() {
  return m_session;
}

void db_manager::add_timeline(std::string const &name) {
  dbo::Transaction tr(m_session);
  
  db_timeline *tmp = new db_timeline(name);
  m_session.add(tmp);
  
  tr.commit();
}

void db_manager::add_token(TREX::transaction::TICK start,
                           TREX::transaction::TICK end,
                           std::string const &tl,
                           std::string const &json) {
  dbo::Transaction tr(m_session);
  
  // get the timeline
  dbo::ptr<db_timeline> timeline = m_session.find<db_timeline>().where("name = ?").bind(tl);
  if( !timeline )
    throw exception("attempted to add an observation to timeline \""+tl+"\" which is not on the database.");
  db_token *obs = new db_token;
  obs->start = start;
  obs->end = end;
  obs->timeline = timeline;
  obs->json = json;
  m_session.add(obs);
  tr.commit();
}

size_t db_manager::get_tokens(std::string const &tl,
                                       bound &min, bound const &max,
                                       std::ostream &out,
                                       size_t max_count) {
  if( max<min || min==transaction::IntegerDomain::plus_inf ) {
    min = transaction::IntegerDomain::plus_inf; // set min to +inf so caller know that he is done
    return 0;
  } else {
    // Build the query : get all the tokens for thsi timeline ordered by their end ...
    dbo::Query< dbo::ptr<db_token> > req = m_session.find<db_token>().where("timeline_name = ?").bind(tl).orderBy("end");
    // with end >= min
    if( !min.isInfinity() )
      req.where("end >= ?").bind(min.value());
    // and start <= max
    if( !max.isInfinity() )
      req.where("start <= ?").bind(max.value());
    // Initiate the transaction
    size_t cpt = 0, count;
    {
      dbo::Transaction tr(m_session);
      // get the max_count first results
      db_tokens result = req.limit(max_count);
      count = result.size();
      // append these results to my stream
      for(db_tokens::const_iterator i=result.begin(); i!=result.end(); ++i, ++cpt) {
        if( cpt>0 )
          out.put(',');
        out<<(*i)->json;
        // update min so next call will omit this value
        min = (*i)->end+1;
      }
      tr.commit();
    }
    if( count<max_count ) {
      // if count is less than our max then we know that we are done
      min = transaction::IntegerDomain::plus_inf; // set min to +inf so caller know that he is done
    }
    return count;
  }
}





