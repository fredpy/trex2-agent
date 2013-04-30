/** @file "Predicate.cc"
 * @brief Precicate class implementation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup transaction
 */  
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
#include "Predicate.hh"

using namespace TREX::utils;
using namespace TREX::transaction;

/*
 * class TREX::transaction::Predicate
 */ 
// structors :

Predicate::Predicate(boost::property_tree::ptree::value_type &node)
  :m_object(parse_attr<Symbol>(node, "on")),
   m_type(parse_attr<Symbol>(node, "pred")) {
  if( m_object.empty() ) 
    throw PredicateException("Empty \"on\" attribute in XML tag");
  if( m_type.empty() )
    throw PredicateException("Empty \"pred\" attribute in XML tag");
  boost::property_tree::ptree::assoc_iterator i,last;
  
  boost::tie(i, last) = node.second.equal_range("Variable");
  for(; last!=i; ++i) {
    Variable var(*i);
    restrictAttribute(var);
  }
}

Predicate::Predicate(Predicate const &other)
  :m_object(other.m_object), m_type(other.m_type), m_vars(other.m_vars) {}

Predicate::~Predicate() {}


// modifiers :

void Predicate::restrictAttribute(Variable const &var) {
  if( !var.isComplete() )
    throw PredicateException("Predicate attribute is not fully defined");
  iterator pos = m_vars.lower_bound(var.name());
  if( end()!=pos && var.name()==pos->second.name() ) {
    pos->second.restrict(var);
  } else {
    // probably need to check if the varaible is valid/OK
    m_vars.insert(pos, std::make_pair(var.name(), var));
  }
}

// observers :

Variable const &Predicate::getAttribute(Symbol const &name) const {
  const_iterator pos = m_vars.find(name);
  if( end()==pos ) 
    throw PredicateException("Attribute \""+name.str()+"\" is unknown");
  return pos->second;
}

std::ostream &Predicate::print_to(std::ostream &out) const {
  std::list<Symbol> vars;
  out<<m_object<<'.'<<m_type;
  out.precision(10);
  listAttributes(vars, false);
  if( !vars.empty() ) {
    out<<'{'<<getAttribute(vars.front());
    for( vars.pop_front(); !vars.empty(); vars.pop_front() )
      out<<", "<<getAttribute(vars.front());
    out<<'}';
  }
  return out;
}

boost::property_tree::ptree Predicate::as_tree(bool all) const {
  boost::property_tree::ptree ret;
  boost::property_tree::ptree &val = ret.add_child(getPredTag().str(), boost::property_tree::ptree());
  
  set_attr(val, "on", object());
  set_attr(val, "pred", predicate());
  if( all ) {
    std::list<Symbol> vars;
    listAttributes(vars, false);

    if( !vars.empty() ) {
      boost::property_tree::ptree &tmp = val.add_child("Variable", boost::property_tree::ptree());
      
      do {
        tmp.push_back(boost::property_tree::ptree::value_type("", getAttribute(vars.front()).as_tree()));
        vars.pop_front();
      } while( !vars.empty() );
    }
  } else if( !m_vars.empty() ) {
    boost::property_tree::ptree &tmp = val.add_child("Variable", boost::property_tree::ptree());

    for(const_iterator i=begin(); end()!=i; ++i) 
      tmp.push_back(boost::property_tree::ptree::value_type("", i->second.as_tree()));
  }
  
  return ret;
}


void Predicate::listAttributes(std::list<TREX::utils::Symbol> &attrs,
			       bool all) const {
  const_iterator i = begin();
  const_iterator const endi = end();
  for( ; endi!=i; ++i )
    if( i->second.isComplete() &&
	( all || !i->second.domain().isFull() ) )
      attrs.push_back(i->first);
}


bool Predicate::consistentWith(Predicate const &other) const {
  if( object()!=other.object() 
      || predicate()!=other.predicate() )
    return false;
  else {
    const_iterator i = begin(), j=other.begin();
    while( end()!=i && other.end()!=j ) {
      if( i->first < j->first )
        i = m_vars.lower_bound(j->first);
      else if( j->first < i->first )
        j = other.m_vars.lower_bound(i->first);
      else {
        if( !i->second.domain().intersect(j->second.domain()) ) {
          //std::cerr << i->second << " != " << j->second << std::endl;
          return false;
        }

        ++i;
        ++j;
      }
    }
    return true;
  }
}
