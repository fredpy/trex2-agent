/* -*- C++ -*- */
/** @file "BasicEnumerated.hh"
 * @brief Implementation of BasicEnumerated
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
#include <sstream>
#include "domain_visitor.hh"

using namespace trex::transaction;
namespace bpt=boost::property_tree;
/*
 * class TREX::transaction::basic_enumerated
 */

// manipulators

void basic_enumerated::complete_parsing(bpt::ptree::value_type &node) {
  bpt::ptree::assoc_iterator i, last;
  
  boost::tie(i, last) = node.second.equal_range("elem");
  if( last==i ) {
    // In case there's no elem childs check if someone specified
    // its value directly
    boost::optional<std::string>
    txt = utils::parse_attr< boost::optional<std::string> >(node, "value");
    if( txt )
      add_string_value(*txt);
  } else {
    for(; last!=i; ++i) {
      boost::optional<std::string>
        txt = utils::parse_attr< boost::optional<std::string> >( i->second,   "value");
      if( txt )
        add_string_value(*txt);
    }
  }
}

// observers

std::string basic_enumerated::element_as_string(size_t i) const {
  std::ostringstream oss;
  print_value(oss, i);
  return oss.str();
}

std::ostream &basic_enumerated::print_domain(std::ostream &out) const {
  size_t i, n=size();
  if( 1==n )
    return print_value(out, 0);
  else {
    print_value(out<<'{', 0);
    for( i=1; i<n; ++i )
      print_value(out<<", ", i);
    return out<<'}';
  }
}

boost::property_tree::ptree basic_enumerated::build_tree() const {
  boost::property_tree::ptree values;
  size_t len = size();
  
  if( len>0 ) {
    boost::property_tree::ptree &tmp = values.add_child("elem", boost::property_tree::ptree());
    
    for(size_t i=0; i<len; ++i) {
      boost::property_tree::ptree e;
      utils::set_attr(e, "value", element_as_string(i));
      tmp.push_back(boost::property_tree::ptree::value_type("", e));
    }
  }
  return values;
}


// manipulators

void basic_enumerated::accept(domain_visitor &visitor) const {
  return visitor.visit(this);
}
