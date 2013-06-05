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
#include "timeline_services.hh"
#include "TimelineHistory.hh"

using namespace TREX::REST;
namespace bp=boost::property_tree;


namespace {
  
  
  transaction::TICK parse_date(std::string const &var, std::string const &val, bool as_date) {
    if( as_date ) {
      try {
        return ptr->get_date(val);
      } catch(std::exception const &e) {
        throw std::runtime_error("Failed to parse "+var+"="+val+" as a date.");
      }
    } else {
      try {
        return boost::lexical_cast<transaction::TICK>(val);
      } catch(boost::bad_lexcal_cast const &e) {
        throw std::runtime_error("Failed to parse "+var+"="+val+" as a tick.");
      }      
    }
  }
  
  void temporal_bounds(Wt::Http::Request const &req,
                       transaction::IntegerDomain::bound &lo,
                       transaction::IntegerDomain::bound &hi) {
    bool as_date = true;
    std::string const *value;
    
    // Gte format option
    value = req.getParameter("format");
    if( NULL!=value ) {
      if( "tick"==*value )
        as_date = false;
      else if( "date"!=*value )
        throw std::runtime_error("Invalid format="+(*format)+". Only accept tick or date");
    }
    // Parse lower and upperbound 
    value = req.getParameter("from");
    if( NULL!=value )
      lo = parse_date("from", *value, as_date);
    value = req.getParameter("to");
    if( NULL!=value )
      hi = parse_date("to", *value, as_date);
  }
                       
}

/*
 * class TREX::REST::timeline_list_service
 */

void timeline_list_service::handleRequest(rest_request const &req,
                                          std::ostream &data,
                                          Wt::Http::Response &ans) {
  boost::shared_ptr<TimelineHistory> ptr(m_entry.lock());
  if( !ptr )
    throw std::runtime_error("Entry point to trex has been destroyed.\n"
                             "This probaly means that trex is terminating.");
  
  transaction::IntegerDomain::bound lo = transaction::IntegerDomain::minus_inf,
  hi = transaction::IntegerDomain::plus_inf;
  
  temporal_bounds(req.request(), lo, hi);
  transaction::IntegerDomain initial(lo, hi);

  ans.setMimeType("application/json");
  std::set<std::string> subset;
  
  rest_request::path_type tmp(req.arg_path());
  
  while( !tmp.empty() )
    subset.insert(tmp.reduce());
  
  bool hidden = false;
  std::string const *full = req.request().getParameter("full");
  
  if( full ) {
    try {
      hidden = boost::lexical_cast<bool>(*full);
    } catch(boost::bad_lexical_cast const &e) {
      // silently ignore for now
    }
  }
  ptr->list_timelines(data, subset, hidden, initial);
}

/*
 * class TREX::REST::timeline_service
 */

void timeline_service::handleRequest(rest_request const &req,
                                     std::ostream &data,
                                     Wt::Http::Response &ans) {
  boost::shared_ptr<TimelineHistory> ptr(m_entry.lock());
  if( !ptr )
    throw std::runtime_error("Entry point to trex has been destroyed.\n"
                             "This probaly means that trex is terminating.");
  
  Wt::Http::ResponseContinuation *cont = req.request().continuation();
  
  transaction::IntegerDomain::bound lo = transaction::IntegerDomain::minus_inf,
  hi = transaction::IntegerDomain::plus_inf,
  now = ptr->now();
  
  bool first = true;
  if( cont ) {
    boost::shared_ptr<transaction::IntegerDomain> range;
    range = boost::any_cast< boost::shared_ptr<transaction::IntegerDomain> >(cont->data());
    
    range->getBounds(lo, hi);
    first = false;
  } else {
    temporal_bounds(req.request(), lo, hi);
    if( lo.isInfinity() ) {
      if( hi>=now )
        lo = now;
    }
    
    ans.setMimeType("application/json");
    transaction::IntegerDomain initial(lo, hi);
    // As it is the start I need to add initial info to the stream
    data<<"{\n \"name\": \""<<req.arg_path().dump()
    <<"\",\"requested_tick_range\": ";
    if( initial.isFull() )
      data<<"{}";
    else
      utils::write_json(data, initial.as_tree(), ptr->fancy());
    data<<",\n \"tokens\": [\n";
  }
  
  ptr->get_tokens(req.arg_path().dump(), lo, hi,
                  data, first, 10);
  if( transaction::IntegerDomain::plus_inf==lo || hi<lo ) {
    data<<" ]\n}";
  } else {
    boost::shared_ptr<transaction::IntegerDomain> range(new transaction::IntegerDomain(lo, hi));
    cont = ans.createContinuation();
    cont->setData(range);
  }
  
}

/*
 * class TREX::REST::goals_service
 */

void goals_service::handleRequest(rest_request const &req,
                                     std::ostream &data,
                                     Wt::Http::Response &ans) {
  boost::shared_ptr<TimelineHistory> ptr(m_entry.lock());
  if( !ptr )
    throw std::runtime_error("Entry point to trex has been destroyed.\n"
                             "This probaly means that trex is terminating.");
  bp::ptree goals = ptr->goals();
  ans.setMimeType("application/json");

  if( goals.empty() )
    // handle spacial case where there is no goal
    data<<"{ \"goals\": [] }";
  else {
    bp::ptree result;
    helpers::json_stream json(data);
    
    result.put_child("goals", goals);
    TREX::utils::write_json(json, result, ptr->fancy());
  }
}


/*
 * class TREX::REST::goal_service
 */

std::string goal_service::file_name() {
  size_t id;
  {
    TREX::utils::SharedVar<size_t>::scoped_lock l(m_counter);
    id = *m_counter;
    *m_counter = id+1;
  }
  
  std::ostringstream oss;

  oss<<m_prefix<<'.'<<id<<".dat";
  return oss.str();
}


void goal_service::handleRequest(rest_request const &req,
                                  std::ostream &data,
                                  Wt::Http::Response &ans) {
  boost::shared_ptr<TimelineHistory> ptr(m_entry.lock());
  if( !ptr )
    throw std::runtime_error("Entry point to trex has been destroyed.\n"
                             "This probaly means that trex is terminating.");
  std::string kind = req.request().method();
  helpers::json_stream json(data);
  
  if( "POST"==kind ) {
    if( "application/json"!=req.request().contentType() )       
      throw std::runtime_error("Data content type must be application/json instead of "+req.request().contentType());
    if( req.request().contentLength()<=0 )
      throw std::runtime_error("Data content is empty");
    std::string file = file_name();
    {
      std::ofstream tmp(file.c_str());
      tmp<<req.request().in().rdbuf();
    }
    TREX::transaction::goal_id g = ptr->add_goal(file);
    ans.setMimeType("application/json");
    utils::write_json(json, ptr->get_goal(g), ptr->fancy());
  } else if( "DELETE"==kind ) {
    std::string id = req.arg_path().dump();
    ans.setMimeType("application/json");
    data<<"{ \"id\": \""<<id<<"\",\n"
      <<"  \"deleted\": \""<<ptr->delete_goal(id)<<"\"\n}";
  } else if( "GET"==kind ) {
    TREX::transaction::goal_id g = ptr->get_goal(req.arg_path().dump());
    ans.setMimeType("application/json");
    utils::write_json(json, ptr->get_goal(g), ptr->fancy());
  } else
    throw std::runtime_error("Do not know how to handle request method "+kind);
}



