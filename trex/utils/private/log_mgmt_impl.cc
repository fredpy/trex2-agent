/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Frederic Py
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
#include "log_mgmt_impl.hh"
#include "../trex_version.hh"


#include <boost/date_time/posix_time/time_formatters.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/tokenizer.hpp>

#include <iomanip>


using namespace TREX::utils;
namespace fs=boost::filesystem;
namespace bpt=boost::posix_time;

using bpt::ptime;

/*
 * class TREX::utils::details::log_mgmt_impl
 */

// structors

log_manager::pimpl::pimpl()
:m_inited(false), m_syslog(m_io.service()), m_strand(m_io.service()) {
  m_io.thread_count(2);
}

log_manager::pimpl::~pimpl() {
  ptime now(bpt::second_clock::universal_time());
  m_syslog(log::null, log::info)<<">>> End of log at "
                                <<bpt::to_simple_string(now)<<" UTC <<<";
}

// observers

std::string log_manager::pimpl::search_path() const {
  std::ostringstream oss;
  bool first = true;
  
  for(path_set::const_iterator i=m_search_path.begin();
      m_search_path.end()!=i; ++i) {
    if( first )
      first = false;
    else
      oss.put(':');
    oss<<i->string();
  }
  return oss.str();
}

// modifiers

bool log_manager::pimpl::set_log_path(log_manager::path_type p) {
  if( !m_inited ) {
    m_path = p;
    m_path.make_preferred();
  }
  return !m_inited;
}

bool log_manager::pimpl::add_search_path(log_manager::path_type p) {
  if( !p.empty() ) {
    p.make_preferred();
    if( p.is_relative() || fs::is_directory(p) ||
       ( fs::is_symlink(p) && fs::is_directory(fs::read_symlink(p)) ) ) {
      m_search_path.push_back(p);
      return true;
    }
  }
  return false;
}

log_manager::path_type log_manager::pimpl::init() {
  if( !m_inited ) {
    if( m_path.empty() )
      create_latest();
    else if( !fs::is_directory(m_path) )
      fs::create_directory(m_path);
    load_search_path();
    strand().post(boost::bind(&pimpl::init_complete, this));
    m_inited = true;
  }
  return m_path;
}

// manipulators

void log_manager::pimpl::flush() {
  if( m_trex_log )
    m_trex_log->flush();
}

bool log_manager::pimpl::locate(std::string const &name,
                                log_manager::path_type &dest) {
  log_manager::path_type loc(name), real_loc;
  bool found;
  
  if( loc.empty() )
    return false;
  
  loc.make_preferred();
  if( fs::is_symlink(loc) )
    real_loc = fs::read_symlink(loc);
  else
    real_loc = loc;
  found = fs::is_regular_file(real_loc);
  
  if( !found && loc.is_relative() ) {
    for(path_set::const_iterator i = m_search_path.begin();
        m_search_path.end()!=i && !found; ++i) {
      log_manager::path_type tmp(*i);
      tmp /= loc;
      if( fs::is_symlink(tmp) )
        real_loc = fs::read_symlink(tmp);
      else
        real_loc = tmp;
      found = fs::is_regular_file(real_loc);
    }
  }
  if( found ) {
    if( !dest.empty() ) {
      dest /= loc;
      dest.make_preferred();
      if( fs::exists(dest) ) {
        // NOTE it would be better to check if it is the same file instead
        m_syslog(log::null, log::warn)<<"A file named \""<<dest.string()
        <<"\" already exists\n\t"
        <<"Keeping initial file instead of overwriting it.";
      } else {
        fs::create_directories(dest.parent_path());
        strand().dispatch(boost::bind(&fs::copy, real_loc, dest));
      }
    }
    dest = real_loc;
  }
  return found;
}

void log_manager::pimpl::create_latest() {
  char *base_dir_name = getenv(LOG_DIR_ENV);
  std::string dir_base;

  if( NULL==base_dir_name ) {
    ERROR_CODE ec = make_error_code(ERRC::no_such_file_or_directory);
    throw SYSTEM_ERROR(ec, "log_manager: $" LOG_DIR_ENV " is not set");
  }
  
  log_manager::path_type base_dir(base_dir_name);
  base_dir.make_preferred();
  {
    // Compute the base log dir name as being YYYY.jjj
    bpt::time_facet *f = new bpt::time_facet("%Y.%j");
    std::ostringstream oss;
    oss.imbue(std::locale(oss.getloc(), f));
    oss<<bpt::second_clock::universal_time();
    dir_base = oss.str();
  }
  log_manager::path_type latest(base_dir);
  size_t last = 0;
  latest /= LATEST_DIR;
  if( fs::exists(latest) ) {
    // Assume that latest is a symlink
    log_manager::path_type real = fs::read_symlink(latest).filename();
    if( real.stem().string()==dir_base ) {
      // Parse the extension minus its inital '.'
      std::istringstream iss(std::string(real.extension().string(), 1));
      iss>>last;
    }
    fs::remove(latest);
  }
  for(size_t i=1; i<MAX_LOG_ATTEMPT; ++i) {
    std::ostringstream oss;
    m_path = base_dir;
    oss<<dir_base<<'.'<<std::setw(2)<<std::setfill('0')<<(last+i);
    m_path /= oss.str();
    if( !fs::exists(m_path) ) {
      fs::create_directory(m_path);
      fs::create_directory_symlink(m_path, latest);
      return;
    }
  }
  // Did not manage to create a directory after MAX_LOG_ATTEMPT
  ERROR_CODE ec = make_error_code(ERRC::value_too_large);
  throw SYSTEM_ERROR(ec, "Too many attempts: clean up your log directory");
}

void log_manager::pimpl::load_search_path() {
  char *path = getenv(SEARCH_ENV);
  if( NULL!=path ) {
    std::string path_str(path);
    static boost::char_delimiters_separator<char> sep(false, "", ":");
    boost::tokenizer<> tok(path_str, sep);

    for(boost::tokenizer<>::iterator it=tok.begin(); tok.end()!=it;++it)
      add_search_path(*it);
  }
}

void log_manager::pimpl::init_complete() {
  log_manager::path_type cfg(m_path), trex(m_path);
  cfg /= "cfg";
  fs::create_directory(cfg);
  
  trex /= TREX_LOG_FILE;
  m_trex_log.reset(new log::out_file(trex.string()));
  m_syslog.direct_connect(m_syslog.stranded(*m_trex_log).track_foreign(m_trex_log));
  m_syslog(log::null, log::info)<<"TREX version "<<TREX::version::full_str();
}



