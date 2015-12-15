/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Frederic Py.
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
#include "token.hh"

using namespace trex::transaction;
namespace utils=trex::utils;
using utils::symbol;

namespace bpt=boost::property_tree;

/*
 * class TREX::transaction::token::attr_comp
 */

bool token::attr_comp::hidden(symbol const &a) const {
  return a.empty() || a.str()[0]=='_';
}

bool token::attr_comp::operator()(symbol const &a, symbol const &b) const {
  if( s_start==b )
    return false;
  else if( s_start==a )
    return true;
  else if( s_duration==b )
    return false;
  else if( s_duration==a )
    return true;
  else if( s_end==b )
    return false;
  else if( s_end==a )
    return true;
  else if( hidden(a) )
    return hidden(b) && !b.empty() && a<b;
  else
    return hidden(b) || a<b;
}

/*
 * class TREX::transaction::token::declare
 */

// structors

token::declare::declare(token::declare::id_param id)
:factory::factory_type::producer(id) {}

// manipulators

token::declare::result_type token::declare::produce
(token::declare::argument_type arg) const {
  token_ref ret = MAKE_SHARED<token>(boost::ref(arg));
  ret->pred_tag(get_id());
  return ret; 
}

/*
 * class TREX::transaction::token
 */

// statics

symbol const token::s_start("start");
symbol const token::s_duration("duration");
symbol const token::s_end("end");

symbol const token::s_failed("Failed");
symbol const token::s_undefined("undefined");


symbol const token::obs_tag("Observation");
symbol const token::goal_tag("Goal");

int_domain const token::s_date_full(int_domain::minus_inf, int_domain::plus_inf);
int_domain const token::s_duration_full(1, int_domain::plus_inf);

// structors

token::token(symbol const &obj, symbol const &pred)
:m_object(obj), m_pred(pred) {
  init_time();
}

token::token(token const &other)
:m_object(other.m_object), m_pred(other.m_pred), m_tag(other.m_tag),
 m_attrs(other.m_attrs) {
  m_start = m_attrs.find(s_start);
  m_duration = m_attrs.find(s_duration);
  m_end = m_attrs.find(s_end);
}

token::token(bpt::ptree::value_type &node)
:m_object(utils::parse_attr<symbol>(node, "on")),
m_pred(utils::parse_attr<symbol>(node, "pred")),
m_tag(node.first) {
  init_time();

  if( object().empty() ) {
    // TODO trigger an exception
  }
  if( predicate().empty() ) {
    // TODO trigger an exception
  }
  bpt::ptree::assoc_iterator i, last;
  boost::tie(i, last) = node.second.equal_range("Variable");
  for(;last!=i; ++i) {
    var v(*i);
    restrict_attribute(v);
  }
}

token &token::operator= (token const &other) {
  m_object = other.m_object;
  m_pred = other.m_pred;
  m_tag = other.m_tag;
  m_attrs = other.m_attrs;
  init_time();
  return *this;
}


token::~token() {}

// observers

bool token::has_attribute(utils::symbol const &name) const {
  return m_attrs.end()!=m_attrs.find(name);
}

var const &token::attribute(utils::symbol const &name) const {
  if( s_start==name )
    return m_start->second;
  else if( s_duration==name )
    return m_duration->second;
  else if( s_end==name )
    return m_end->second;
  else {
    attr_set::const_iterator i = m_attrs.find(name);
    if( m_attrs.end()==i ) {
      // TODO throw and error
    }
    return i->second;
  }
}

bool token::consistent_with(token const &other) const {
  // Check  that the token is the same
  if( object()!=other.object() ||
      predicate()!=other.predicate() )
    return false;
  else {
    attr_set::const_iterator i = m_attrs.begin(), j = other.m_attrs.begin();
    attr_set::value_compare cmp(m_attrs.value_comp());
    
    while( m_attrs.end()!=i && other.m_attrs.end()!=j ) {
      if( cmp(*i,*j) ) {
        // *i does not exist in other => we assume that it just means
        // that it has not been set
        // Advance i to the closest to j
        i = m_attrs.lower_bound(j->first);
      } else if( cmp(*j,*i) ) {
        // *j does not exist in *this => we assume that it just means
        // that it has not been set
        // Advance j to the closest to i
        j = m_attrs.lower_bound(i->first);
      } else {
        if( !i->second.domain().intersect(j->second.domain()) ) {
          // domains of same attribute do not intersect
          return false;
        }
        // this variable is consitent move on to the next ones
        ++i;
        ++j;
      }
    }
    // All attributes are consistent => these tokens are consitent
    return true;
  }
}
bool token::is_temporal(token::attr_iterator const &i) const {
  return !m_attrs.key_comp()(i->first, s_end);
}

bool token::is_full(token::attr_iterator const &i) const {
  if( m_duration==i )
    return i->second.domain().equals(s_duration_full);
  else
    return i->second.domain().is_full();
}

token::attr_iterator token::attr_begin(bool all) const {
  if( all )
    return m_attrs.begin();
  else
    return m_attrs.upper_bound(s_end);
}

token::attr_iterator token::attr_end(bool all) const {
  if( all )
    return m_attrs.end();
  else
    return m_attrs.lower_bound(utils::symbol());
}

void token::list_attributes(std::list<symbol> &attrs, bool all) const {
  attr_iterator const last = attr_end(all);
  
  for(attr_set::const_iterator i = attr_begin(all); last!=i; ++i)
    attrs.push_back(i->first);
}

bpt::ptree token::as_tree(bool full) const {
  bpt::ptree ret;
  bpt::ptree &val = ret.add_child(pred_tag().str(), bpt::ptree());
  
  set_attr(val, "on", object());
  set_attr(val, "pred", predicate());
  
  bpt::ptree vars;
  attr_iterator const last = attr_end(full);
  
  for(attr_iterator i = attr_begin(full);
      last!=i; ++i) {
    if( !is_full(i) ) {
      vars.push_back(bpt::ptree::value_type("", i->second.as_tree()));
    }
  }
  if( !vars.empty() )
    val.add_child("Variable", vars);
  return ret;
}

// manipulators

bool token::starts_after(int_domain::base_type date,
                         int_domain::base_type delay) {
  int_domain test_window(date+delay, int_domain::plus_inf),
    cstr_window(date, int_domain::plus_inf);
  if( m_start->second.domain().intersect(test_window) ) {
    ERROR_CODE ec;
    restrict_start(cstr_window, ec);
    if( ec ) {
      std::ostringstream oss;
      oss<<object()<<"."<<predicate()<<".start *= "<<test_window;
      throw SYSTEM_ERROR(ec, oss.str());
    }
    return true;
  }
  return false;
}

void token::restrict_time(int_domain const &s,
                          int_domain const &d,
                          int_domain const &e) {
  ERROR_CODE ec;
  restrict_time(s, d, e, ec);
  if( ec ) {
    std::ostringstream oss;
    oss<<object()<<'.'<<predicate()<<".restrict("<<s<<", "<<d<<", "<<e<<")";
    throw SYSTEM_ERROR(ec, oss.str());
  }
 
}


void token::restrict_time(int_domain const &s,
                          int_domain const &d,
                          int_domain const &e,
                          ERROR_CODE &ec) {
  int_domain::bound lo_s, hi_s, lo_d, hi_d, lo_e, hi_e;
  int_domain::bound lo, hi;
  
  bool s_changed = false, d_changed = false, e_changed = false;
  
  if( !( m_start->second.domain().intersect(s) &&
         m_duration->second.domain().intersect(d) &&
        m_end->second.domain().intersect(e) ) ) {
    ec = domain_error_code(domain_error::empty_domain);
    return;
  }
  
  // Get new initial bounds for start domain
  m_start->second.typed_domain<int_domain>().get_bounds(lo_s,hi_s);
  s.get_bounds(lo, hi);
  lo = lo_s.max(lo);
  hi = hi_s.min(hi);
  
  if( lo!=lo_s || hi!=hi_s ) {
    s_changed = true;
    lo_s = lo;
    hi_s = hi;
  }
    
  
  // Get new initial bounds for duration domain
  m_duration->second.typed_domain<int_domain>().get_bounds(lo_d,hi_d);
  d.get_bounds(lo, hi);
  lo = lo_d.max(lo);
  hi = hi_d.min(hi);
  if( lo!=lo_d || hi!=hi_d ) {
    d_changed = true;
    lo_d = lo;
    hi_d = hi;
  }
  
  
  // Get new initial bounds for end domain
  m_end->second.typed_domain<int_domain>().get_bounds(lo_e,hi_e);
  e.get_bounds(lo, hi);
  lo = lo_e.max(lo);
  hi = hi_e.min(hi);
  if( lo!=lo_e || hi!=hi_e ) {
    e_changed = true;
    lo_e = lo;
    hi_e = hi;
  }
  
  // Now propagate the constraint start+duration==end
  
  // end = end "inter" start+duration
  hi = hi_e.min(hi_s.plus(hi_d, hi_e));
  lo = lo_e.max(lo_s.plus(lo_d, lo_e));
  if( lo!=lo_e || hi!=hi_e ) {
    e_changed = true;
    lo_e = lo;
    hi_e = hi;
  }
  if( lo_e>hi_e ) {
    ec = domain_error_code(domain_error::empty_domain);
    return;
  }
  
  // start = start "inter" end-duration
  hi = hi_s.min(hi_e.minus(lo_d, hi_s));
  lo = lo_s.max(lo_e.minus(hi_d, lo_s));
  if( lo!=lo_s || hi!=hi_s ) {
    s_changed = true;
    lo_s = lo;
    hi_s = hi;
  }
  if( lo_s>hi_s ) {
    ec = domain_error_code(domain_error::empty_domain);
    return;
  }
  
  // duration = duration "inter" start-end
  hi = hi_d.min(hi_e.minus(lo_s, hi_d));
  lo = lo_d.max(lo_e.minus(hi_s, lo_d));
  if( lo!=lo_d || hi!=hi_d ) {
    d_changed = true;
    lo_d = lo;
    hi_d = hi;
  }
  if( lo_d>hi_d ) {
    ec = domain_error_code(domain_error::empty_domain);
    return;
  }
  ec = domain_error_code(domain_error::ok);
  
  if( s_changed ) {
    m_start->second.restrict_with(int_domain(lo_s, hi_s), ec);
    assert(!ec); // must be always true at this point
    m_updated(*this, m_start->second);
  }
  if( d_changed ) {
    m_duration->second.restrict_with(int_domain(lo_d, hi_d), ec);
    assert(!ec); // must be always true at this point
    m_updated(*this, m_duration->second);
  }
  if( e_changed ) {
    m_end->second.restrict_with(int_domain(lo_e, hi_e), ec);
    assert(!ec); // must be always true at this point
    m_updated(*this, m_end->second);
  }
}

void token::restrict_attribute(var const &v, ERROR_CODE &ec) {
  symbol const &name = v.name();
  if( s_start==name )
    restrict_start(v.typed_domain<int_domain>(), ec);
  else if( s_duration==name )
    restrict_duration(v.typed_domain<int_domain>(), ec);
  else if( s_end==name )
    restrict_end(v.typed_domain<int_domain>(), ec);
  else {
    bool updated;
    attr_set::iterator i = constrain(v, updated, ec);
    if( updated )
      m_updated(*this, i->second);
  }
}


void token::restrict_attribute(var const &v) {
  ERROR_CODE ec;
  restrict_attribute(v, ec);
  if( ec ) {
    std::ostringstream oss;
    oss<<object()<<'.'<<predicate()<<".restrict("<<v<<")";
    throw SYSTEM_ERROR(ec, oss.str());
  }
}

void token::merge_with(token const &other, ERROR_CODE &ec) {
  if( object()!=other.object() || predicate()!=other.predicate() ) {
    ec = domain_error_code(domain_error::incompatible_tokens);
  } else {
    attr_set::iterator i = m_attrs.begin();
    attr_set::const_iterator j = attr_begin(false); // skip start,duration,end
                                                    // as we want to handle them
                                                    // differently later
    attr_set::value_compare cmp(m_attrs.value_comp());
    std::list<var> merged;
  
  
    while( m_attrs.end()!=i && other.m_attrs.end()!=j ) {
      if( cmp(*i, *j) )
        i = m_attrs.lower_bound(j->first);
      else if( cmp(*j, *i) ) {
        merged.push_back(j->second);
        ++j;
      } else {
        var tmp = i->second;
        bool ret = tmp.restrict_with(j->second, ec);
        if( ec )
          return;
        else if( ret )
          merged.push_back(tmp); // domain was updated => queue it
        ++i; ++j;
      }
    }
    restrict_time(other.start(), other.duration(), other.end(), ec);
    if( !ec ) {
      ERROR_CODE must_be_ok;
      bool ignore;
      while( !merged.empty() ) {
        i = constrain(merged.front(), ignore, must_be_ok);
        assert(!must_be_ok);
        merged.pop_front();
        m_updated(*this, i->second);
      }
      for( ;other.m_attrs.end()!=j; ++j) {
        i = constrain(j->second, ignore, must_be_ok);
        assert(!must_be_ok);
        m_updated(*this, i->second); // Notify on attribute addition
      }
    }
  }
}

void token::merge_with(token const &other) {
  ERROR_CODE ec;
  merge_with(other, ec);
  if( ec ) {
    std::ostringstream oss;
    oss<<(*this)<<" *= "<<other;
    throw SYSTEM_ERROR(ec, oss.str());
  }
}


token::attr_set::iterator token::constrain(var const &v,
                                           bool &updated,
                                           ERROR_CODE &ec) {
  attr_set::iterator pos = m_attrs.lower_bound(v.name());
  updated = false;

  if( m_attrs.end()!=pos && v.name()==pos->first ) {
    if( pos->second.restrict_with(v, ec) )
      updated = true;
  } else {
    pos = m_attrs.insert(pos, std::make_pair(v.name(), v));
    ec = domain_error_code(domain_error::ok);
    updated = true;
  }
  return pos;
}

void token::init_time() {
  bool ignore;
  ERROR_CODE ec;
  
  m_start = constrain(var(s_start, s_date_full), ignore, ec);
  m_duration = constrain(var(s_duration, s_duration_full), ignore, ec);
  m_end = constrain(var(s_end, s_date_full), ignore, ec);
}

// related

std::ostream &trex::transaction::operator<<(std::ostream &out, token const &tok) {
  bool first = true;
  
  out<<tok.object()<<'.'<<tok.predicate();
  out.precision(10);
  token::attr_iterator const last = tok.attr_end(false);
  

  for(token::attr_iterator i=tok.attr_begin(false);
      last!=i; ++i) {
    if( !tok.is_full(i) ) {
      if( first ) {
        out.write("{ ", 2);
        first=false;
      } else
        out.write(", ", 2);
      out<<i->second;
    }
  }
  if( !first )
    out.put('}');
  return out;
}

namespace {

  token::declare obs_parser(token::obs_tag);
  token::declare goal_parser(token::goal_tag);

}







