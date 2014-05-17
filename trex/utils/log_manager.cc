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

// trex headers
#include "log_manager.hh"
#include "shared_var.hh"
#include "TREXversion.hh"
#include "asio_runner.hh"

// C++ standard headers
#include <ctime>
#include <cstring>

#include <iomanip>

// boost libraries
#include <boost/tokenizer.hpp>
#include <boost/date_time/posix_time/time_formatters.hpp>

#include <boost/system/error_code.hpp>

using namespace boost::filesystem;
namespace bpt=boost::posix_time;
using bpt::ptime;

namespace TREX {
  namespace utils {
    namespace details {
      
      class mgmt_impl :boost::noncopyable {
      public:
        mgmt_impl();
        ~mgmt_impl();
        
        log::text_log &syslog() {
          return m_syslog;
        }
        void flush() {
          if( m_trex_log )
            m_trex_log->flush();
        }
        
        asio_runner &asio() {
          return m_io;
        }
        boost::asio::strand &strand() {
          return m_strand;
        }
        
        bool set_log_path(log_manager::path_type p);
        bool add_search_path(log_manager::path_type p);
        
        log_manager::path_type init() {
          if( !m_inited ) {
            if( m_path.empty() )
              create_latest();
            else if( !is_directory(m_path) )
              create_directory(m_path);
            load_search_path();
            strand().post(boost::bind(&mgmt_impl::init_complete, this));
            m_inited = true;
          }
          return m_path;
        }
        
        bool locate(std::string const &name,
                    log_manager::path_type &dest);
        
        std::string search_path() const {
          std::ostringstream oss;
          bool first = true;
          for(std::list<log_manager::path_type>::const_iterator i=m_search_path.begin();
              m_search_path.end()!=i; ++i) {
            if( first )
              first = false;
            else
              oss.put(':');
            oss<<i->string();
          }
          return oss.str();
        }
        
      private:
        void create_latest();
        void load_search_path();
        
        bool m_inited;
        
        log_manager::path_type m_path;
        asio_runner            m_io;
        log::text_log          m_syslog;
        boost::asio::strand     m_strand;
        
        SHARED_PTR<log::out_file> m_trex_log;
        std::list<log_manager::path_type> m_search_path;
        
        void init_complete();
      };
 
      mgmt_impl::mgmt_impl():m_inited(false),
        m_syslog(m_io.service()), m_strand(m_io.service()) {
          m_io.thread_count(2);
      }
      
      mgmt_impl::~mgmt_impl() {
        ptime now(bpt::second_clock::universal_time());
        m_syslog(log::null, log::info)<<">>> End of log at "
        <<bpt::to_simple_string(now)<<" UTC <<<";
      }
      
      bool mgmt_impl::set_log_path(log_manager::path_type p) {
        if( !m_inited ) {
          m_path = p;
          m_path.make_preferred();
        }
        return !m_inited;
      }

      void mgmt_impl::init_complete() {
        log_manager::path_type cfg(m_path), trex(m_path);
        cfg /= "cfg";
        create_directory(cfg);
        
        trex /= TREX_LOG_FILE;
        m_trex_log.reset(new log::out_file(trex.string()));
        m_syslog.direct_connect(m_syslog.stranded(*m_trex_log).track_foreign(m_trex_log));
        m_syslog(log::null, log::info)<<"TREX version "
          <<TREX::version::full_str();
      }
      
      void mgmt_impl::create_latest() {
        char *base_dir_name = getenv(LOG_DIR_ENV);
        char dated_dir[16];

        if( NULL==base_dir_name ) {
          ERROR_CODE ec = make_error_code(ERRC::no_such_file_or_directory);
          throw SYSTEM_ERROR(ec, "log_manager: $" LOG_DIR_ENV " is no set");
        }
        
        log_manager::path_type base_dir(base_dir_name);
        base_dir.make_preferred();

        // compute the base name using YYYY.jjj format
        dated_dir[15] = '\0';
        time_t cur_time;
        time(&cur_time);
        strftime(dated_dir, 15, "%Y.%j", gmtime(&cur_time));
        std::string dir_base(dated_dir);
        
        log_manager::path_type latest(base_dir);
        size_t last = 0;

        latest /= LATEST_DIR;
        if( exists(latest) ) {
          // Assumes that latest is a symlink
          log_manager::path_type real = read_symlink(latest).filename();
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
            return; // sucessfully done
          }
        }
        // Did not manage to create a directory after MAX_LOG_ATTEMPT
        ERROR_CODE ec = make_error_code(ERRC::value_too_large);
        throw SYSTEM_ERROR(ec, "Too many attempts: clean up your log directory");
      }

      void mgmt_impl::load_search_path() {
        char *path = getenv(SEARCH_ENV);
        if( NULL!=path ) {
          std::string path_str(path);
          static boost::char_delimiters_separator<char> sep(false, "", ":");
          boost::tokenizer<> tok(path_str, sep);
          
          for(boost::tokenizer<>::iterator it=tok.begin(); tok.end()!=it;++it)
            add_search_path(*it);
        }
      }
      
      bool mgmt_impl::add_search_path(log_manager::path_type p) {
        if( !p.empty() ) {
          p.make_preferred();
          if( p.is_relative() || is_directory(p) ||
              ( is_symlink(p) && is_directory(read_symlink(p)) ) ) {
            m_search_path.push_back(p);
            return true;
          }
        }
        return false;
      }

      bool mgmt_impl::locate(std::string const &name,
                             log_manager::path_type &dest) {
        log_manager::path_type loc(name), real_loc;
        bool found;
        
        if( loc.empty() )
          return false;
        
        loc.make_preferred();
        if( is_symlink(loc) )
          real_loc = read_symlink(loc);
        else
          real_loc = loc;
        
        found = is_regular_file(real_loc);
        
        if( !found && loc.is_relative() ) {
          std::list<log_manager::path_type>::const_iterator i = m_search_path.begin();
          for( ; m_search_path.end()!=i && !found ; ++i) {
            log_manager::path_type tmp(*i);
            tmp /= loc;
            if( is_symlink(tmp) )
              real_loc = read_symlink(tmp);
            else
              real_loc = tmp;
            found = is_regular_file(real_loc);
          }
        }
        if( found ) {
          if( !dest.empty() ) {
            dest /= loc;
            dest.make_preferred();
            if( exists(dest) ) {
              // Note: would be better to check if this is the same file
              m_syslog(log::null, log::warn)<<"A file named \""
              <<dest.string()<<" already exists\n\t"
              <<"I won't overwrite the previous file.";
            } else {
              create_directories(dest.parent_path());
              strand().post(boost::bind(boost::filesystem::copy, real_loc,
                                        dest));
            }
          }
          std::swap(dest,real_loc);
          return true;
        }
        return false;
      }

    }
  }
}

using namespace TREX::utils; 


/*
 * class LogManager
 */

log_manager::log_manager():m_impl(new details::mgmt_impl) {}

log_manager::~log_manager() {}

log_manager::path_type log_manager::log_path() {
  boost::function<path_type ()> fn = boost::bind(&details::mgmt_impl::init, m_impl.get());
  return strand_run(m_impl->strand(), fn);
}

bool log_manager::log_path(std::string const &path) {
  path_type p(path);
  boost::function<bool ()> fn(boost::bind(&details::mgmt_impl::set_log_path,
                                          m_impl.get(), p));
  return strand_run(m_impl->strand(), fn);
}

bool log_manager::add_search_path(std::string const &path) {
  path_type p(path);
  boost::function<bool ()> fn(boost::bind(&details::mgmt_impl::add_search_path,
                                          m_impl.get(), p));
  return strand_run(m_impl->strand(), fn);
}

std::string log_manager::search_path() {
  boost::function<std::string ()> fn(boost::bind(&details::mgmt_impl::search_path,
                                                 m_impl.get()));
  return strand_run(m_impl->strand(), fn);
}

void log_manager::flush() {
  m_impl->flush();
}


log::text_log &log_manager::syslog() {
  return m_impl->syslog();
}

size_t log_manager::thread_count() const {
  return m_impl->asio().thread_count();
}

size_t log_manager::thread_count(size_t n, bool override_hw) {
  return m_impl->asio().thread_count(n, override_hw);
}

boost::asio::io_service &log_manager::service() {
  return m_impl->asio().service();
}

log_manager::path_type log_manager::locate(std::string const &file_name,
                                            bool &found) {
  path_type ret;
  boost::function<bool ()> fn(boost::bind(&details::mgmt_impl::locate, m_impl.get(),
                                          file_name, boost::ref(ret)));
  found = strand_run(m_impl->strand(), fn);
  return ret;
}

log_manager::path_type log_manager::use(std::string const &file_name,
                                         bool &found) {
  path_type ret = cfg_path();
  boost::function<bool ()> fn(boost::bind(&details::mgmt_impl::locate, m_impl.get(),
                                          file_name, boost::ref(ret)));
  found = strand_run(m_impl->strand(), fn);
  if( found )
    syslog(log::null, log::info)<<" - Using "<<ret.string();
  return ret;
}

log_manager::path_type log_manager::log_file(std::string const &short_name) {
  path_type ret(log_path());
  ret /= short_name;
  ret.make_preferred();
  create_directories(ret.parent_path());
  return ret;
}


