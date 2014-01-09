/** @file "Variable.cc"
 * @brief Variable class implementation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup domains
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
# include <algorithm>
# include <iterator>

# include "Variable.hh"

using namespace TREX::utils;
using namespace TREX::transaction;

namespace bp=boost::property_tree;

/*
 * class TREX::transaction::Variable
 */

// statics :

singleton::use<DomainBase::xml_factory> Variable::s_dom_factory;

DomainBase *Variable::clone
(boost::call_traits<Variable::domain_ptr>::param_type dom) {
  if( !dom )
    return NULL;
  else
    return dom->copy();
}

// structors :

Variable::Variable() {}

Variable::Variable(symbol const &name, DomainBase *dom)
  :m_name(name), m_domain(dom) {}

Variable::Variable(symbol const &name, DomainBase const &dom) 
  :m_name(name), m_domain(dom.copy()) {
  if( m_name.empty() )
    throw VariableException("Empty variable names are not allowed");
}

Variable::Variable(Variable const &other) 
  :m_name(other.m_name), m_domain(clone(other.m_domain)) {}

Variable::Variable(boost::property_tree::ptree::value_type &node)
  :m_name(parse_attr<symbol>(node.second, "name")) {
  if( m_name.empty() )
    throw bp::ptree_bad_data("Variable name is empty", node);
  
  boost::property_tree::ptree::iterator i = node.second.begin();
  if( !s_dom_factory->iter_produce(i, node.second.end(), m_domain) )
    throw bp::ptree_bad_data("Missing variable domain on XML tag", node);
}

Variable::~Variable() {}

// Modifiers :

Variable &Variable::operator= (Variable const &other) {
  m_name = other.m_name;
  m_domain.reset(clone(other.m_domain));
  return *this;
}

Variable &Variable::restrict(DomainBase const &dom) {
  if( !m_domain )
    m_domain.reset(dom.copy());
  else 
    m_domain->restrictWith(dom);
  return *this;
}

Variable &Variable::restrict(Variable const &var) {
  if( var.m_name.empty() ) {
    if( var.m_domain )
      restrict(*(var.m_domain));
  } else if( m_name.empty() ) {
    if( var.m_domain )
      restrict(*(var.m_domain));
    m_name = var.m_name;
  } else {
    if( m_name!=var.m_name ) {
      throw VariableException("Cannot merge variables with different names.");
    }
    if( var.m_domain )
      restrict(*(var.m_domain));
  }
  return *this;
}

// Observers :

DomainBase const &Variable::domain() const {
  if( !m_domain ) 
    throw VariableException("Variable's domain cannot be accessed.");
  return *m_domain;
} 

boost::property_tree::ptree Variable::as_tree() const {
  boost::property_tree::ptree ret;
  
  if( m_domain ) {
    ret = m_domain->as_tree();
    set_attr(ret, "type", m_domain->getTypeName());
  } else
    set_attr(ret, "type", "null");
  set_attr(ret, "name", name());
  return ret;
}


