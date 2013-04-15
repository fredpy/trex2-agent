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
#include "DomainVisitor.hh"

using namespace TREX::transaction;
namespace bpt=boost::property_tree;
/*
 * class TREX::transaction::BasicEnumerated
 */

// structors 

// modifiers :

void BasicEnumerated::completeParsing(bpt::ptree::value_type &node) {
  bpt::ptree::assoc_iterator i, last;
  
  boost::tie(i, last) = node.second.equal_range("elem");
  for(; last!=i; ++i) {
    boost::optional<std::string> 
      txt = TREX::utils::parse_attr< boost::optional<std::string> >( i->second, "value");
    if( txt )
      addTextValue(*txt);
  }
}

// observers :

std::string BasicEnumerated::getStringValue(size_t i) const {
  std::ostringstream oss;
  print_value(oss, i);
  return oss.str();
}

std::ostream &BasicEnumerated::print_domain(std::ostream &out) const {
  size_t i, size = getSize();
  if( 1==size ) 
    return print_value(out, 0);
  else {
    print_value(out<<'{', 0);
    for( i=1; i<size; ++i ) 
      print_value(out<<", ", i);
    return out<<'}';
  }
}

std::ostream &BasicEnumerated::toXml(std::ostream &out, size_t tabs) const {
  std::ostream_iterator<char> pad(out);
  std::fill_n(pad, tabs, ' '); 
  size_t i, size = getSize();
  out<<'<'<<getTypeName();
  if( 0==size )
    return out<<"/>";
  else {
    out<<">\n";
    for( i=0 ; i<size; ++i ) {
      bpt::ptree elem;
      elem.put("elem.<xmlattr>.value", getStringValue(i));
      std::fill_n(pad, tabs+1, ' ');
      // very hacky at this stage
      write_xml_element(out, bpt::ptree::key_type(), elem, -1,
			bpt::xml_parser::xml_writer_settings<bpt::ptree::key_type::value_type>());
    }
    std::fill_n(pad, tabs, ' '); 
    return out<<"</"<<getTypeName()<<'>';
  }
}

std::ostream &BasicEnumerated::toJSON(std::ostream &out, size_t tabs) const {
  std::ostream_iterator<char> pad(out);
  size_t i, size = getSize();
  bool first = true;
  
  std::fill_n(pad, tabs, ' ');
  out<<'\"'<<getTypeName()<<"\": ";
  if( size>0 ) {
    out<<"{\n";
    std::fill_n(pad, tabs+1, ' ');
    out<<"\"elem\": [";
    for(i=0; i<size; ++i) {
      if( first )
        first = false;
      else
        out.put(',');
      out.put('\n');
      std::fill_n(pad, tabs+2, ' ');
      out<<"{ \"value\": \""<<getStringValue(i)<<"\" }";
    }
    out.put('\n');
    std::fill_n(pad, tabs+1, ' ');
    out<<"]\n";
    std::fill_n(pad, tabs, ' ');
    return out.put('}');
  } else
    return out<<"null";
}


// manipulators

void BasicEnumerated::accept(DomainVisitor &visitor) const {
  return visitor.visit(this);
}
