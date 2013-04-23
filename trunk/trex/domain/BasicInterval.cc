/* -*- C++ -*- */
/** @file "BasicInterval.hh"
 * @brief Implementation of BasicInterval
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
#include "DomainVisitor.hh"

using namespace TREX::transaction;
namespace bpt=boost::property_tree;

// modifiers 

void BasicInterval::completeParsing(bpt::ptree::value_type &node) {
  // Try for a singleton first 
  boost::optional<std::string> 
    val = TREX::utils::parse_attr< boost::optional<std::string> >(node.second, "value");
  if( val ) {
    parseSingleton(*val);
  } else {
    boost::optional< std::string >
      lo = TREX::utils::parse_attr< boost::optional<std::string> >(node.second, "min"),
      hi = TREX::utils::parse_attr< boost::optional<std::string> >(node.second, "max");

    if( lo ) 
      parseLower(*lo);
    if( hi )
      parseUpper(*hi);
  }
}

// observers 
    
std::string BasicInterval::getStringLower() const {
  std::ostringstream oss;
  oss.precision(16);
  oss.setf(oss.fixed);
  print_lower(oss);
  return oss.str();
}

std::string BasicInterval::getStringUpper() const {
  std::ostringstream oss;
  oss.precision(16);
  oss.setf(oss.fixed);
  print_upper(oss);
  return oss.str();
}

std::ostream &BasicInterval::print_domain(std::ostream &out) const {
  //out.setf(out.fixed);
  if( isSingleton() ) 
    return print_lower(out);
  else
    return print_upper(print_lower(out<<'[')<<", ")<<']';
}

boost::property_tree::ptree BasicInterval::build_tree() const {
  boost::property_tree::ptree info;
  if( isSingleton() )
    TREX::utils::set_attr(info, "value", getStringSingleton());
  else {
    if( hasLower() )
      TREX::utils::set_attr(info, "min", getStringLower());
    if( hasUpper() )
      TREX::utils::set_attr(info, "min", getStringUpper());
  }
  return info;
}


// manipulators

void BasicInterval::accept(DomainVisitor &visitor) const {
  return visitor.visit(this);
}
