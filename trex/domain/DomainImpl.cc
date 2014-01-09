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

#include "BooleanDomain.hh"
#include "FloatDomain.hh"
#include "IntegerDomain.hh"
#include "StringDomain.hh"
#include "EnumDomain.hh"

using namespace TREX::transaction;
using namespace TREX::utils;


namespace {
  /** @brief Declaration of the @c int domain
   * @ingroup domains
   */
  DomainBase::xml_factory::declare<IntegerDomain> int_decl  ("int");
  /** @brief Declaration of the @c float domain 
   * @ingroup domains
   */
  DomainBase::xml_factory::declare<FloatDomain>   float_decl("float");
  /** @brief Declaration of the @c bool domain 
   * @ingroup domains
   */
  DomainBase::xml_factory::declare<BooleanDomain> bool_decl ("bool");
 
  /** @brief Declaration of the @c string domain 
   * @ingroup domains
   */
  DomainBase::xml_factory::declare<StringDomain> declStr("string");  


  /** @brief Declaration of the @c enum domain 
   * @ingroup domains
   */
  DomainBase::xml_factory::declare<EnumDomain> decl_enum("enum");  
}

symbol const BooleanDomain::type_name("bool");
symbol const IntegerDomain::type_name("int");
symbol const FloatDomain::type_name("float");
symbol const StringDomain::type_name("string");
symbol const EnumDomain::type_name("enum");

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


BooleanDomain::BooleanDomain(boost::property_tree::ptree::value_type &node)
  :BasicInterval(node), m_full(true) {
  boost::optional<int> val = parse_attr< boost::optional<int> >(node, "value");
  if( val ) {
    m_val = (0!=val);
    m_full = false;
  } else
    completeParsing(node); // to handle the case where someon used min/max attributes
}

std::ostream &BooleanDomain::print_lower(std::ostream &out) const {
  if( m_full || !m_val )
    return out<<false;
  else
    return out<<true;
}


void BooleanDomain::parseSingleton(std::string const &val) {
  int v = boost::lexical_cast<int>(val);
  bool bv = (0!=v);
  if( m_full ) {
    m_full = false;
    m_val = bv;
  } else if( m_val!=bv )
    throw EmptyDomain(*this, "setting to singleton resulted in boolean empty domain");

}

void BooleanDomain::parseLower(std::string const &val) {
  int v = boost::lexical_cast<int>(val);
  bool bv = (0!=v);
  if( bv ) {
    if( m_full ) {
      m_full = false;
      m_val = bv;
    } else if( m_val!=bv )
      throw EmptyDomain(*this, "setting to lower bound to true resulted in boolean empty domain");
  }
}


void BooleanDomain::parseUpper(std::string const &val)  {
  int v = boost::lexical_cast<int>(val);
  bool bv = (0!=v);
  if( !bv ) {
    if( m_full ) {
      m_full = false;
      m_val = bv;
    } else if( m_val!=bv )
      throw EmptyDomain(*this, "setting to upper bound to false resulted in boolean empty domain");
  }
}


std::ostream &BooleanDomain::print_upper(std::ostream &out) const {
  if( m_full || m_val )
    return out<<true;
  else
    return out<<false;
}


std::ostream &BooleanDomain::toXml(std::ostream &out,
				   size_t tabs) const {
    std::fill_n(std::ostream_iterator<char>(out), tabs, ' ');
    out<<"<bool";
    if( !m_full )
        out<<" value=\""<<m_val<<'\"';
    out<<"/>";
    return out;
}


bool BooleanDomain::intersect(DomainBase const &other) const {
  if( other.getTypeName()!=getTypeName() )
    return false;
  else {
    BooleanDomain const &ref = dynamic_cast<BooleanDomain const &>(other);
    
    return m_full || ref.m_full || m_val==ref.m_val;
  }
}

bool BooleanDomain::equals(DomainBase const &other) const {
  if( other.getTypeName()!=getTypeName() )
    return false;
  else {
    BooleanDomain const &ref = dynamic_cast<BooleanDomain const &>(other);
    if( m_full )
      return ref.m_full;
    else 
      return !ref.m_full && m_val==ref.m_val;
  } 
}
      
DomainBase &BooleanDomain::restrictWith(DomainBase const &other) {
  if( other.getTypeName()!=getTypeName() )
    throw EmptyDomain(*this, "Incompatible types");
  else {
    BooleanDomain const &ref = dynamic_cast<BooleanDomain const &>(other);
    if( m_full ) {
      m_full = ref.m_full;
      m_val = ref.m_val;
    } else if( !ref.m_full && m_val!=ref.m_val ) 
      throw EmptyDomain(*this, "intersection is empty.");
  }
  return *this;
}

