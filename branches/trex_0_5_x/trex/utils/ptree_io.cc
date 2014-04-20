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
#include "ptree_io.hh"

// Need to indicate to spirit to be thread safe
# define BOOST_SPIRIT_THREADSAFE

# include <boost/property_tree/json_parser.hpp>
# include <boost/property_tree/xml_parser.hpp>

namespace bp=boost::property_tree;
namespace xml=boost::property_tree::xml_parser;
namespace json=boost::property_tree::json_parser;

namespace {
  
  bool remove_arrays(std::string const &name, bp::ptree &p) {
    if( p.empty() )
      return false;
    else {
      bp::ptree tmp;
      p.swap(tmp);
      
      if( tmp.size()==tmp.count(std::string()) ) {
        for(bp::ptree::iterator i=tmp.begin(); tmp.end()!=i; ++i) {
          remove_arrays(i->first, i->second);
          p.push_back(bp::ptree::value_type(name, i->second));
        }
        return true;
      } else {
        for(bp::ptree::iterator i=tmp.begin(); tmp.end()!=i; ++i) {
          if( remove_arrays(i->first, i->second) )
            p.insert(p.end(), i->second.begin(), i->second.end());
          else
            p.push_back(*i);
        }
        return false;
      }
    }
  }
    
}
  
void TREX::utils::flatten_json_arrays(bp::ptree &p) {
  remove_arrays("", p);
}

void TREX::utils::flatten_xml_attrs(bp::ptree &p) {
  if( !p.empty() ) {
    bp::ptree tmp;
    p.swap(tmp);
    for(bp::ptree::iterator i=tmp.begin(); tmp.end()!=i; ++i) {
      if( "<xmlattr>"==i->first )
        p.insert(p.end(), i->second.begin(), i->second.end());
      else {
        flatten_xml_attrs(i->second);
        p.push_back(*i);
      }
    }
  }
}

void TREX::utils::read_xml(std::istream &in, bp::ptree &p) {
  xml::read_xml(in, p, xml::no_comments|xml::trim_whitespace);
}

void TREX::utils::write_xml(std::ostream &out, bp::ptree p, bool header) {
  // Clean up the mess for json
  flatten_json_arrays(p);
  if( header )
    xml::write_xml(out, p);
  else {
    // Hacky way to get ridd of the XML header
    xml::write_xml_element(out, bp::ptree::key_type(), p, -1,
                           xml::xml_writer_settings<bp::ptree::key_type::value_type>());
  } 
}

void TREX::utils::read_json(std::istream &in, bp::ptree &p) {
  json::read_json(in, p);
  // clean that up as it impedes my parsing
  flatten_json_arrays(p);
}

void TREX::utils::write_json(std::ostream &out, boost::property_tree::ptree p, bool fancy) {
  // Clean up the mess for xml
  flatten_xml_attrs(p);
  // fancy print
  json::write_json(out, p, fancy);
}


using namespace TREX::utils;

std::ostream &ptree_convertible::to_xml(std::ostream &out) const {
  write_xml(out, as_tree(), false);
  return out;
}

std::ostream &ptree_convertible::to_json(std::ostream &out) const {
  write_json(out, as_tree());
  return out;
}






