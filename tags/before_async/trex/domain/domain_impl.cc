/** @file "DomainImpl.cc
 * @brief Various domains implementation
 *
 * This file implements the code for domains which are part of the TREX
 * core.
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
#include <iterator>
#include <algorithm>

#include <cmath>

#include "boolean_domain.hh"
#include "float_domain.hh"
#include "int_domain.hh"
#include "string_domain.hh"
#include "enum_domain.hh"

using namespace TREX::transaction;
using namespace TREX::utils;


namespace {
  /** @brief Declaration of the @c int domain
   * @ingroup domains
   */
  abstract_domain::factory::declare<int_domain> int_decl  ("int");
  /** @brief Declaration of the @c float domain 
   * @ingroup domains
   */
  abstract_domain::factory::declare<float_domain>   float_decl("float");
  /** @brief Declaration of the @c bool domain 
   * @ingroup domains
   */
  abstract_domain::factory::declare<boolean_domain> bool_decl ("bool");
 
  /** @brief Declaration of the @c string domain 
   * @ingroup domains
   */
  abstract_domain::factory::declare<string_domain> decl_str("string");


  /** @brief Declaration of the @c enum domain 
   * @ingroup domains
   */
  abstract_domain::factory::declare<enum_domain> decl_enum("enum");
}

symbol const boolean_domain::type_str("bool");
symbol const int_domain::type_str("int");
symbol const float_domain::type_str("float");
symbol const string_domain::type_str("string");
symbol const enum_domain::type_str("enum");

double TREX::transaction::round(double d, size_t places) {
  double factor = pow(10, places);
  return ::round(d*factor)/factor;
}

double TREX::transaction::floor(double d, size_t places) {
  double factor = pow(10, places);
  return std::floor(d*factor)/factor;  
}

double TREX::transaction::ceil(double d, size_t places) {
  double factor = pow(10, places);
  return std::ceil(d*factor)/factor;  
}


boolean_domain::boolean_domain(boost::property_tree::ptree::value_type &node)
  :basic_interval(node), m_full(true) {
  boost::optional<int> val = parse_attr< boost::optional<int> >(node, "value");
  if( val ) {
    m_val = (0!=val);
    m_full = false;
  } else
    complete_parsing(node); // to handle the case where someon used min/max attributes
}

std::ostream &boolean_domain::print_lower(std::ostream &out) const {
  if( m_full || !m_val )
    return out<<false;
  else
    return out<<true;
}


void boolean_domain::parse_singleton(std::string const &val) {
  int v = boost::lexical_cast<int>(val);
  bool bv = (0!=v);
  if( m_full ) {
    m_full = false;
    m_val = bv;
  } else if( m_val!=bv )
    throw SYSTEM_ERROR(domain_error_code(domain_error::empty_domain),
                       "parsing singleton(bool)");
}

void boolean_domain::parse_lower(std::string const &val) {
  int v = boost::lexical_cast<int>(val);
  bool bv = (0!=v);
  if( bv ) {
    if( m_full ) {
      m_full = false;
      m_val = bv;
    } else if( m_val!=bv )
      throw SYSTEM_ERROR(domain_error_code(domain_error::empty_domain),
                         "parsing lower bound(bool)");
  }
}


void boolean_domain::parse_upper(std::string const &val)  {
  int v = boost::lexical_cast<int>(val);
  bool bv = (0!=v);
  if( !bv ) {
    if( m_full ) {
      m_full = false;
      m_val = bv;
    } else if( m_val!=bv )
      throw SYSTEM_ERROR(domain_error_code(domain_error::empty_domain),
                         "parsing upper bound(bool)");
  }
}


std::ostream &boolean_domain::print_upper(std::ostream &out) const {
  if( m_full || m_val )
    return out<<true;
  else
    return out<<false;
}


boost::property_tree::ptree boolean_domain::build_tree() const {
  boost::property_tree::ptree info;
  
  if( !m_full )
    set_attr(info, "value", m_val);
  return info;
}

bool boolean_domain::intersect(abstract_domain const &other) const {
  if( other.type_name()!=type_name() )
    return false;
  else {
    boolean_domain const &ref = dynamic_cast<boolean_domain const &>(other);
    
    return m_full || ref.m_full || m_val==ref.m_val;
  }
}

bool boolean_domain::equals(abstract_domain const &other) const {
  if( other.type_name()!=type_name() )
    return false;
  else {
    boolean_domain const &ref = dynamic_cast<boolean_domain const &>(other);
    if( m_full )
      return ref.m_full;
    else 
      return !ref.m_full && m_val==ref.m_val;
  } 
}
      
bool boolean_domain::restrict_with(abstract_domain const &other,
                                   ERROR_CODE &ec) {
  if( other.type_name()!=type_name() )
    ec = domain_error_code(domain_error::incompatible_types);
  else {
    boolean_domain const &ref = dynamic_cast<boolean_domain const &>(other);
    ec = domain_error_code(domain_error::ok);
    if( !ref.m_full ) {
      if( m_full ) {
        m_full = false;
        m_val = ref.m_val;
        return true;
      } else if( m_val!=ref.m_val )
        ec = domain_error_code(domain_error::empty_domain);
    }
  }
  return false;
}


