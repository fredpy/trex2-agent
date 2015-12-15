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
#include "domain_visitor.hh"

using namespace trex::transaction;
namespace bpt=boost::property_tree;

/*
 * class TREX::transaction::basic_interval
 */

// manipulators

void basic_interval::complete_parsing(bpt::ptree::value_type &node) {
  // Try for a singleton first 
  boost::optional<std::string> 
    val = utils::parse_attr< boost::optional<std::string> >(node.second, "value");
  if( val ) {
    parse_singleton(*val);
  } else {
    boost::optional< std::string >
      lo = utils::parse_attr< boost::optional<std::string> >(node.second, "min"),
      hi = utils::parse_attr< boost::optional<std::string> >(node.second, "max");

    if( lo ) 
      parse_lower(*lo);
    if( hi )
      parse_upper(*hi);
  }
}

// observers 
    
std::string basic_interval::lower_as_string() const {
  std::ostringstream oss;
  oss.precision(16);
  oss.setf(oss.fixed);
  print_lower(oss);
  return oss.str();
}

std::string basic_interval::upper_as_string() const {
  std::ostringstream oss;
  oss.precision(16);
  oss.setf(oss.fixed);
  print_upper(oss);
  return oss.str();
}

std::ostream &basic_interval::print_domain(std::ostream &out) const {
  if( is_singleton() )
    return print_lower(out);
  else
    return print_upper(print_lower(out<<'[')<<", ")<<']';
}

boost::property_tree::ptree basic_interval::build_tree() const {
  boost::property_tree::ptree info;
  if( is_singleton() )
    utils::set_attr(info, "value", get_singleton_as_string());
  else {
    if( has_lower() )
      utils::set_attr(info, "min", lower_as_string());
    if( has_upper() )
      utils::set_attr(info, "max", upper_as_string());
  }
  return info;
}


// manipulators

void basic_interval::accept(domain_visitor &visitor) const {
  return visitor.visit(this);
}
