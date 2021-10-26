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
#include "private/Schema.hh"
#include "trex/europa/EuropaPlugin.hh"
#include "trex/europa/Assembly.hh"

#include <trex/utils/XmlUtils.hh>

// include plasma header as system files in order to disable warnings
# define TREX_PP_SYSTEM_FILE <PLASMA/Debug.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/Error.hh>
# include <trex/europa/bits/system_header.hh>

#include <boost/algorithm/string/replace.hpp>
#include <boost/regex.hpp>


using namespace TREX::europa;
namespace utils=TREX::utils;

namespace xml = boost::property_tree::xml_parser;

/*
 * class TREX::europa::details::Schema
 */

// statics



std::set<std::string> details::Schema::includes(std::istream &is) {
  boost::regex inc_exp("# *include +(?:(?:<([^>]+)>)|(?:\"([^\"]+)\"))");
  std::set<std::string> result;
  std::string line;
  
  while( std::getline(is, line) ) {
    boost::match_results<std::string::const_iterator> 
      matches;
    if( boost::regex_match(line, matches, inc_exp) ) {
      if( matches[1].matched )
        result.insert(matches[1]); // was a <>
      else if( matches[2].matched )
        result.insert(matches[2]); // was a ""
    }
  }
  return result;
}


// structors 

details::Schema::Schema() {
  bool found;
  
  std::string cfg_dbg = m_log->use("Debug.cfg", found);
  
  if( found ) {
    std::ifstream cfg(cfg_dbg.c_str());
    DebugMessage::readConfigFile(cfg);
  } else 
    m_log->syslog("europa", TREX::utils::log::warn)<<"No europa Debug.cfg found:\n\t"
    <<"Keeping default europa verbosity level.";
}

// manipulators

void details::Schema::registerComponents(Assembly const &assembly) {
  for(std::set<EuropaPlugin *>::const_iterator i=m_plugins.begin();
      m_plugins.end()!=i; ++i)
    (*i)->registerComponents(assembly);
}

//void details::Schema::setStream(std::ofstream &out, std::string const &name) {
//  out.open(m_log->file_name(name).c_str());
//  setStream(out);
//}

void details::Schema::setStream(std::ostream &out) {
  Error::doThrowExceptions();
  DebugMessage::setStream(out);
  Error::setStream(out);
}

std::string const &details::Schema::nddl_path() {
  if( m_path.empty() ) {
    bool found;
    std::string config;
    std::ostringstream opath;
    
    // First add TREX_PATH
    opath<<'.';

    for(utils::LogManager::path_iterator i=m_log->begin(); m_log->end()!=i; ++i)
      opath<<':'<<i->string();
    
    // Now extract infotmnation for NDDL config file
    config = m_log->use("NDDL.cfg", found);
    if( !found ) {
      config = m_log->use("temp_nddl_gen.cfg", found);
      if( !found ) 
	throw EuropaException("Unable to locate NDDL.cfg or temp_nddl_gen.cfg");
    }
    
    boost::property_tree::ptree cfg;
    read_xml(config, cfg, xml::no_comments|xml::trim_whitespace);
    if( cfg.empty() ) 
      throw EuropaException(config+" does not appear to be a valid xml tree.");
    else if( cfg.size()!=1 )
      throw EuropaException(config+" have multiple xml roots.");
    
    boost::property_tree::ptree::assoc_iterator i,last;
    boost::tie(i, last) = cfg.front().second.equal_range("include");
    for(; last!=i; ++i) {
      std::string path = utils::parse_attr<std::string>(*i, "path");
      boost::replace_all(path, ";", ":");
      opath<<':'<<path;
    }
    
    m_path = opath.str();
  }
  return m_path;
}

void details::Schema::registerFunction(Assembly const &assembly, EUROPA::CFunction *fn) {
  assembly.constraint_engine()->getCESchema()->registerCFunction(fn->getId());
}

std::string details::Schema::use(std::string file, bool &found) {
  include_map::iterator i = m_includes.find(file);
  if( m_includes.end()==i ) {
    std::filesystem::path p = m_log->use(file, found);
    if( found ) {
      m_includes.insert(include_map::value_type(file, p));
      
      std::ifstream cif(p.string().c_str());
      std::set<std::string> incs = includes(cif);
      cif.close();
      for(std::set<std::string>::const_iterator f=incs.begin();
        incs.end()!=f; ++f) {
        bool fincl;
        use(*f, fincl);
      }
    }
    return p.string();
  } else {
    found = true;
    return i->second.string();
  }
}

// modifiers

void details::Schema::registerPlugin(EuropaPlugin &pg) {
  m_plugins.insert(&pg);
}

void details::Schema::unregisterPlugin(EuropaPlugin &pg) {
  m_plugins.erase(&pg);
}

/*
 * class TREX::europa::EuropaPlugin
 */

// structors 

EuropaPlugin::EuropaPlugin() {
  m_schema->registerPlugin(*this);
}

EuropaPlugin::~EuropaPlugin() {
  m_schema->unregisterPlugin(*this);
}

// manipulators

void EuropaPlugin::declareFunction(Assembly const &assembly, EUROPA::CFunction *fn) {
  debugMsg("trex:declare", "New function "<<fn->getName().toString()<<" declared");
  m_schema->registerFunction(assembly, fn);
}

