/** @file "LogManager.cc"
 * @brief LogManager implementation
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
#include <time.h>

#include <boost/tokenizer.hpp>
#include <boost/date_time/posix_time/time_formatters.hpp>

#include <cerrno>
#include <cstring>
#include <iomanip>

#include "LogManager.hh"
#include "TREXversion.hh"

using namespace TREX::utils; 
using namespace boost::filesystem;


/*
 * class LogManager
 */ 

// structors :

LogManager::LogManager():m_inited(false), m_syslog(m_io.service()), m_level(TREX_LOG_LEVEL) {
  // Capture standard output and error in TREX.log
  m_out.reset(new log::log_pipe(m_syslog, std::cout, "_cout_", ""));
  m_log.reset(new log::log_pipe(m_syslog, std::clog, "_clog_", ""));
  m_err.reset(new log::log_pipe(m_syslog, std::cerr, "_cerr_", "ERROR"));
}

LogManager::~LogManager() {
  boost::posix_time::ptime now(boost::posix_time::second_clock::universal_time());
  
  syslog("", log::info)<<">>> End of log at "<<boost::posix_time::to_simple_string(now)<<" UTC <<<";
  m_out.reset();
  m_log.reset();
  m_err.reset();
}

// methods :


LogManager::path_type LogManager::locate(std::string const &file, bool &found) const {
  path_type ret(file);
  
  if( file.empty() )
    found = false;
  else {
    ret.make_preferred();
    
    found = is_regular_file(ret);
    
    if( !found && ret.is_relative() ) {
      path_iterator i = m_searchPath.begin();
      for(; m_searchPath.end()!=i; ++i ) {
        path loc(*i);
        loc /= ret; 
        path real_loc(loc);
        if( is_symlink(loc) )
          real_loc = read_symlink(loc);
        
        found = is_regular_file(real_loc);
	if( found )
	  return loc;
      }
    }
  }
  return ret;
}

bool LogManager::addSearchPath(std::string const &path) {
  if( !path.empty() ) {
    path_type p(path);
    p.make_preferred();
    
    if( p.is_relative() || is_directory(p) ||
       ( is_symlink(p) && is_directory(read_symlink(p)) ) ) {
      m_searchPath.push_back(p);
      return true;
    }
  }
  return false;
}

void LogManager::loadSearchPath() {
  char *home = getenv(TREX_ENV);
  if( NULL!=home ) {
    std::string home_path(home);
    // Right now I am adding $TREX_HOME as it is
    // In  the future I may have to add $TREX_HOME/extra
    // -- or whatever I decide -- instead
    addSearchPath(home_path);
  }
  char *path = getenv(SEARCH_ENV);
  if( NULL!=path ) {
    // column separated list
    std::string path_str(path);
    static boost::char_delimiters_separator<char> sep(false, "", ":");
    boost::tokenizer<> tok(path_str, sep);
    for(boost::tokenizer<>::iterator beg=tok.begin(); beg!=tok.end();++beg) {
      addSearchPath(*beg);
    }
  }
}

LogManager::path_type const &LogManager::logPath() {
  SharedVar<bool>::scoped_lock lock(m_inited);

  if( !*m_inited ) {
    if( m_path.empty() ) {
      createLatest();
    } else if( !is_directory(m_path) ) { 
      create_directory(m_path);
    }
    path_type cfg(m_path), trex_log(m_path);
    cfg /= "cfg";
    create_directory(cfg);
    
    trex_log /= TREX_LOG_FILE;
    m_trex_log.reset(new log::out_file(trex_log.string()));
    m_syslog.direct_connect(m_syslog.stranded(*m_trex_log).track(m_trex_log));
    
    thread_count(2);
    
    syslog("", log::null)<<"TREX version "<<TREX::version::str();
    loadSearchPath();
    *m_inited = true;
  }
  return m_path;
}

void LogManager::createLatest() {
  char *base_dir_name = getenv(LOG_DIR_ENV);
  char dated_dir[16];
  
  if( NULL==base_dir_name ) {
    throw ErrnoExcept("LogManager: $"LOG_DIR_ENV" is not set");
  }
  path_type base_dir(base_dir_name);
  base_dir.make_preferred();
  
  // compute the base name using YYYY.jjj format
  dated_dir[15] = '\0';
  time_t cur_time;
  time(&cur_time);
  strftime(dated_dir, 15, "%Y.%j", gmtime(&cur_time));
  std::string dir_base(dated_dir);
  
  path_type latest(base_dir);
  size_t last = 0;
  
  latest /= LATEST_DIR;
  if( exists(latest) ) {
    // I assume that lates is a symlink
    path_type real = read_symlink(latest).filename();
    if( real.stem().string()==dir_base ) {
      std::istringstream iss(std::string(real.extension().string(), 1));
      iss>>last;
    }
    remove(latest);
  }
  for(unsigned i=1; i<MAX_LOG_ATTEMPT; ++i) {
    std::ostringstream oss;
    m_path = base_dir;
    oss<<dir_base<<'.'<<std::setw(2)<<std::setfill('0')<<(last+i);
    
    m_path /= oss.str();
    if( !exists(m_path) ) {
      create_directory(m_path);
      create_directory_symlink(m_path, latest);
      return;
    }
  }
  throw ErrnoExcept("LogManager", "Too many attempts ... clean up your "
		    "log directory");
}

LogManager::path_type LogManager::file_name(std::string const &short_name) {
  path_type ret(logPath());
  ret /= short_name;
  ret.make_preferred();
  create_directories(ret.parent_path());
  return ret;
}

LogManager::path_type LogManager::getCfgPath() {
  return file_name("cfg");
}

std::string LogManager::use(std::string const &file_name, bool &found) {
  path_type located = locate(file_name, found);
  if( found ) {
    path_type dest(getCfgPath()), tmp(file_name), src(located);
    if( tmp.is_relative() ) 
      dest /= tmp;
    else dest /= src.filename();
    
    syslog("", log::info)<<" - Using file "<<located;
    if( exists(dest) )
      syslog("", log::warn)<<"A file with this name already exists in the cfg log."
		      <<"\n\tI won't overwrite the previous file.";
    else {
      if( is_symlink(located) ) 
        src = read_symlink(located);
      create_directories(dest.parent_path());
      copy(src, dest);
    }
  }
  return located.string();
}

bool LogManager::setLogLevel(LogLevel lvl) {
  SharedVar<bool>::scoped_lock guard(m_inited);
  if( !*m_inited ) {
    m_level = lvl;
    return true;
  }
  syslog("", log::error)<<"Cannot change verbosity level after init."<<std::endl;
  return false;
}

bool LogManager::setLogPath(std::string const &path) {
  SharedVar<bool>::scoped_lock guard(m_inited);
  if( !*m_inited ) {
    m_path = path;
    return true;
  }
  syslog("", log::error)<<"Cannot change log path after init."<<std::endl;
  return false;
}


