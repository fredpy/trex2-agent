/* -*- C++ -*- */
/** @file "XmlUtils.cc"
 * @brief XML utilities implmentation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
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
#include "XmlUtils.hh"

using namespace TREX::utils;
namespace xml = boost::property_tree::xml_parser;
namespace bp = boost::property_tree;

boost::optional<bp::ptree const &> TREX::utils::internals::find_attr(bp::ptree const &pt,
                                                                     std::string const &path) {
  boost::optional<bp::ptree const &> ret = pt.get_child_optional("<xmlattr>."+path);
  
  if( !ret )
    ret = pt.get_child_optional(path);
  return ret;
}


namespace {
  singleton::use<log_manager> s_log;
}

void TREX::utils::ext_xml(boost::property_tree::ptree &tree, std::string const &conf, bool ahead) {
  boost::optional<std::string> name = parse_attr< boost::optional<std::string> >(tree, conf);
  
  if( name ) {
    bool found;
    std::string file = s_log->use(*name, found).string();
    
    if( !found ) {
      ERROR_CODE ec = make_error_code(ERRC::no_such_file_or_directory);
      throw SYSTEM_ERROR(ec, "Unable to locate file \""+(*name)+"\"");
    }
    
    boost::property_tree::ptree pt;
    read_xml(file, pt, xml::no_comments|xml::trim_whitespace);
    if( pt.empty() )
      throw bp::ptree_error("Xml file \""+file+"\" is empty.");
    if( pt.size()==1 )
      pt = pt.front().second;

    boost::optional<boost::property_tree::ptree &> attrs = tree.get_child_optional("<xmlattr>");
    
    if( attrs ) {
      boost::property_tree::ptree::assoc_iterator attrs2 = pt.find("<xmlattr>");
      if( attrs2!=pt.not_found() ) {
        if( ahead )
          attrs->insert(attrs->begin(), attrs2->second.begin(), attrs2->second.end());
        else
          attrs->insert(attrs->end(), attrs2->second.begin(), attrs2->second.end());
        pt.erase(pt.to_iterator(attrs2));
      }
    }
    

    if( ahead )
      tree.insert(tree.begin(), pt.begin(), pt.end());
    else 
      tree.insert(tree.end(), pt.begin(), pt.end());
  }
}

