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

# include "var.hh"

using namespace TREX::utils;
namespace bp=boost::property_tree;

namespace TREX {
  namespace transaction {
    
    class var::impl :public ptree_convertible {
    public:
      impl(symbol const &name, abstract_domain *dom)
      :m_name(name), m_domain(dom) {
        if( m_name.empty() )
          throw VariableException("Empty var names are not allowed");
      }
      impl(symbol const &name, abstract_domain const &dom)
      :m_name(name), m_domain(dom.copy()) {
        if( m_name.empty() )
          throw VariableException("Empty var names are not allowed");
      }
      impl(bp::ptree &node);
      impl(impl const &other):m_name(other.name()) {
        if( other.has_domain() )
          m_domain.reset(other.m_domain->copy());
      }
      ~impl() {}
      
      symbol const &name() const {
        return m_name;
      }
      bool has_domain() const {
        return m_domain;
      }
      abstract_domain &domain() {
        return *m_domain;
      }
      
      void restrict_with(abstract_domain const &dom) {
        if( !m_domain )
          m_domain.reset(dom.copy());
        else
          m_domain->restrict_with(dom);
      }
      
      bp::ptree as_tree() const;
    
      static impl *clone(UNIQ_PTR<impl> const &other) {
        if( NULL==other.get() )
          return NULL;
        else
          return new impl(*other);
      }
  
    private:
      static singleton::use<abstract_domain::factory> s_dom_fact;
      typedef abstract_domain::factory::returned_type dom_ref;
      
      symbol  m_name;
      dom_ref m_domain;
      
      impl() DELETED;
      
    }; // TREX::transaction::var::impl
  

    singleton::use<abstract_domain::factory> var::impl::s_dom_fact;
    
    /*
     * class TREX::transaction::var::impl
     */
    var::impl::impl(bp::ptree &node)
    :m_name(parse_attr<symbol>(node, "name")) {
      if( m_name.empty() )
        throw bp::ptree_bad_data("var name is empty", node);
      boost::property_tree::ptree::iterator i = node.begin();
      if( !s_dom_fact->iter_produce(i, node.end(), m_domain) )
        throw bp::ptree_bad_data("Missing variable domain on XML tag", node);
    }

    bp::ptree var::impl::as_tree() const {
      boost::property_tree::ptree ret;
      
      if( has_domain() ) {
        ret = m_domain->as_tree();
        set_attr(ret, "type", m_domain->type_name());
      } else
        set_attr(ret, "type", "null");
      set_attr(ret, "name", name());
      return ret;
    }

    std::ostream &operator<<(std::ostream &out, var const &v) {
      if( v.is_complete() )
        out<<v.name()<<'='<<v.domain();
      else
        out<<"<?"<<v.name()<<'>';
      return out;
    }

    
  } // TREX::transaction
} // TREX

using namespace TREX::transaction;


/*
 * class TREX::transaction::Variable
 */

// structors :

var::var() {}

var::var(symbol const &name, abstract_domain const &dom)
:m_impl(new var::impl(name, dom)) {}

var::var(var const &other)
:m_impl(var::impl::clone(other.m_impl)) {}

var::var(boost::property_tree::ptree::value_type &node)
:m_impl(new var::impl(node.second)) {}

var::~var() {}

// observers:

bool var::is_complete() const {
  return NULL!=m_impl.get() && m_impl->has_domain();
}

symbol var::name() const {
  if( NULL==m_impl.get() )
    return symbol();
  return m_impl->name();
}

abstract_domain const &var::domain() const {
  if( !is_complete() )
    throw VariableException("Variable's domain cannot be accessed");
  return m_impl->domain();
}


// Modifiers :

var &var::operator= (var const &other) {
  m_impl.reset(var::impl::clone(other.m_impl));
  return *this;
}

var &var::restrict_with(abstract_domain const &dom) {
  if( NULL==m_impl.get() )
    throw VariableException("Cannot restrict a variable with no name");
  m_impl->restrict_with(dom);
  return *this;
}

var &var::restrict_with(var const &v) {
  if( NULL==m_impl.get() )
    return operator=(v);
  else if( NULL!=v.m_impl.get() ) {
    if( name()!=v.name() )
      throw VariableException("Cannot merge varibales with different names");
    m_impl->restrict_with(v.domain());
  }
  return *this;
}

// Observers :

bp::ptree var::as_tree() const {
  if( is_complete() )
    return m_impl->as_tree();
  return bp::ptree();
}


