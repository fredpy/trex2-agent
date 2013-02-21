/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2013, MBARI.
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
#include <trex/utils/Symbol.hh>
#include <trex/utils/TREXversion.hh>
#include <trex/utils/LogManager.hh>

#include <boost/python.hpp>
#include <boost/iterator/transform_iterator.hpp>

#include <functional>

using namespace boost::python;

namespace {
  
  class log_wrapper {
  public:
    explicit log_wrapper(std::string const &name)
    :m_name(name) {}
    explicit log_wrapper(TREX::utils::Symbol const &name)
    :m_name(name) {}
    ~log_wrapper() {}
    
    
    std::string get_log_dir() const {
      return m_log->logPath().string();
    }
    void set_log_dir(std::string const &path) {
      m_log->setLogPath(path);
    }
    void log(TREX::utils::Symbol const &what, std::string const &msg) {
      get_log_dir();
      m_log->syslog(m_name, what)<<msg;
    }
    void info(std::string const &msg) {
      log(TREX::utils::log::info, msg);
    }
    void warn(std::string const &msg) {
      log(TREX::utils::log::warn, msg);
    }
    void error(std::string const &msg) {
      log(TREX::utils::log::error, msg);
    }
    
    std::string use(std::string fname) {
      bool found;
      fname = m_log->use(fname, found);
      if( found )
        return fname;
      return std::string();
    }
        
    std::string path() const {
      get_log_dir();
      std::ostringstream ret;
      bool not_first = false;
      for(TREX::utils::LogManager::path_iterator i=m_log->begin(); m_log->end()!=i; ++i) {
        if( not_first )
          ret.put(':');
        else
          not_first = true;
        ret<<i->string();
      }
      return ret.str();
    }
    
    bool add_path(std::string dir) {
      return m_log->addSearchPath(dir);
    }
    
  private:
    TREX::utils::Symbol m_name;
    TREX::utils::SingletonUse<TREX::utils::LogManager> m_log;
  };
      
}


BOOST_PYTHON_MODULE(trex_py)
{
  
  class_<TREX::utils::Symbol>("symbol", "Unique instance symbolic value",
                              init<optional<std::string> >())
   .def("empty", &TREX::utils::Symbol::empty)
   .def("__len__", &TREX::utils::Symbol::length)
   .def(self == self)
   .def(self != self)
   .def(self < self)
   .def(self > self)
   .def(self <= self)
   .def(self >= self)
   .def("__str__", &TREX::utils::Symbol::str, return_value_policy<copy_const_reference>())
  ;
  
  class_<TREX::version>("version", "Version information about trex", no_init)
  .add_static_property("major", &TREX::version::major)
  .add_static_property("minor", &TREX::version::minor)
  .add_static_property("release", &TREX::version::release)
  .add_static_property("is_rc", &TREX::version::is_release_candidate)
  .add_static_property("rc", &TREX::version::rc_number)
  .add_static_property("str", &TREX::version::str)
  ;
  
  class_<log_wrapper>("log", "Logging for trex", init<std::string>())
  .def(init<TREX::utils::Symbol>())
  .add_property("dir", &log_wrapper::get_log_dir, &log_wrapper::set_log_dir)
  .def("use_file", &log_wrapper::use)
  .def("info", &log_wrapper::info)
  .def("warn", &log_wrapper::warn)
  .def("error", &log_wrapper::error)
  .add_property("path", &log_wrapper::path)
  .def("add_path", &log_wrapper::add_path)
  ;
  
} // BOOST_PYTHON_MODULE(trex_py)
