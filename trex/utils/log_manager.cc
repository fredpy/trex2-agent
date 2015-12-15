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
#include "private/log_mgmt_impl.hh"
#include "shared_var.hh"


// boost libraries

using namespace boost::filesystem;
namespace bpt=boost::posix_time;
using bpt::ptime;


using namespace trex::utils;


/*
 * class LogManager
 */

log_manager::log_manager():m_impl(new pimpl) { }

log_manager::~log_manager() {}

log_manager::path_type log_manager::log_path() {
  boost::function<path_type ()> fn = boost::bind(&pimpl::init, m_impl.get());
  return strand_run(m_impl->strand(), fn);
}

bool log_manager::log_path(std::string const &path) {
  path_type p(path);
  boost::function<bool ()> fn(boost::bind(&pimpl::set_log_path,
                                          m_impl.get(), p));
  return strand_run(m_impl->strand(), fn);
}

bool log_manager::add_search_path(std::string const &path) {
  path_type p(path);
  boost::function<bool ()> fn(boost::bind(&pimpl::add_search_path,
                                          m_impl.get(), p));
  return strand_run(m_impl->strand(), fn);
}

std::string log_manager::search_path() {
  boost::function<std::string ()> fn(boost::bind(&pimpl::search_path,
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
  boost::function<bool ()> fn(boost::bind(&pimpl::locate,
                                          m_impl.get(),
                                          file_name, boost::ref(ret)));
  found = strand_run(m_impl->strand(), fn);
  return ret;
}

log_manager::path_type log_manager::use(std::string const &file_name,
                                         bool &found) {
  path_type ret = cfg_path();
  boost::function<bool ()> fn(boost::bind(&pimpl::locate,
                                          m_impl.get(),
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


