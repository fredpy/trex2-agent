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
#include <boost/signals2/shared_connection_block.hpp>

#include <functional>

using namespace boost::python;

namespace {
  
  class scoped_gil_release:boost::noncopyable {
  public:
    scoped_gil_release() {
      m_gil_state = PyGILState_Ensure();
    }
    ~scoped_gil_release() {
      PyGILState_Release(m_gil_state);
    }
    
  private:
    PyGILState_STATE m_gil_state;
  };
  
  TREX::utils::SingletonUse<TREX::utils::LogManager> s_log;

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
    
    TREX::utils::Symbol m_name;
  private:
    TREX::utils::SingletonUse<TREX::utils::LogManager> m_log;
  };
  
  class py_log_handler {
  public:
    py_log_handler() {}
    virtual ~py_log_handler() {
      disconnect();
    }
    
    bool connected() const {
      return m_conn.connected();
    }
    void disconnect() {
      m_conn.disconnect();
    }
    
    virtual void new_entry(TREX::utils::log::entry::pointer e) =0;
    
  protected:
    void handle_interface(TREX::utils::log::entry::pointer e) {
      scoped_gil_release protect;
      boost::signals2::shared_connection_block lock(m_conn);
      try {
        new_entry(e);
      } catch(error_already_set const &e) {
        disconnect();
        // Extract error message for logging
        PyObject *ptype, *pvalue, *ptrace;
        PyErr_Fetch(&ptype, &pvalue, &ptrace);

        std::string msg = extract<std::string>(pvalue);
        
        m_log->syslog("_python_", TREX::utils::log::error)<<msg;
        // Reestablish error and display it in python
        PyErr_Restore(ptype, pvalue, ptrace);
        PyErr_Print();
      } catch(...) {
        disconnect();
        m_log->syslog("python", TREX::utils::log::error)<<"Unknown exception during python callback.";
      }
    }
    
    void init() {
      m_conn = m_log->on_new_log(boost::bind(&py_log_handler::handle_interface, this, _1),true);
    }
    
  private:
    TREX::utils::log::text_log::connection             m_conn;
    TREX::utils::SingletonUse<TREX::utils::LogManager> m_log;
  };
  
  struct py_log_handler_wrap:py_log_handler, wrapper<py_log_handler> {
  public:
    py_log_handler_wrap() {
      py_log_handler::init();
    }
    
    void new_entry(TREX::utils::log::entry::pointer e);
  };
  
}


// Python API for TREX::utils
//   this basic API only exposes the bare minimum. Namely:
//     - trex.symbol  for TREX::utils::Symbol
//     - trex.version access to TREX version information
//     - trex.log     basic/barebone access to TREX::utils::LogManager
BOOST_PYTHON_MODULE(trex)
{
  
  PyEval_InitThreads();
    
  // trex.symbol class
  //   can be created with a string
  //   can be compared to each other with (==,!=,<,>,<=,=>)
  //   can be checked if empty()
  //   supports str(s) and len(s)
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
  
  // python string can be implicitly converted into trex.symbol
  implicitly_convertible<std::string, TREX::utils::Symbol>();
  
  // trex.version class
  // A class with only static read only properties
  //   trex.version.major : major version number
  //   trex.version.minor : minor version number
  //   trex.version.release : release number
  //   trex.version.is_rc : indicates if it is a release candidate
  //   trex.version.rc : release candidate number (or 0 if is_rc is False)
  //   trex.version.str : A string value for this version of TREX
  class_<TREX::version>("version", "Version information about trex", no_init)
  .add_static_property("major", &TREX::version::major)
  .add_static_property("minor", &TREX::version::minor)
  .add_static_property("release", &TREX::version::release)
  .add_static_property("is_rc", &TREX::version::is_release_candidate)
  .add_static_property("rc", &TREX::version::rc_number)
  .add_static_property("str", &TREX::version::str)
  ;
  
  // Log message entry
  //   - no constructor (produced internally on log messages)
  //   - is_dated : indicate if the entry has a date
  //   - date() : the date of the entry (if is_dated)
  //   - source() : the source of the entry (symbol)
  //   - kind() : the type of the entry (symbol, either "INFO", "WARN", ...)
  //   - content() : the message content as a string
  class_<TREX::utils::log::entry, TREX::utils::log::entry::pointer>("log_entry",
                                                                    "A single log entry message", no_init)
  .add_property("is_dated", &TREX::utils::log::entry::is_dated, "Check if dated")
  .def("date", &TREX::utils::log::entry::date, return_value_policy<copy_const_reference>())
  .def("source", &TREX::utils::log::entry::source, return_internal_reference<>())
  .def("kind", &TREX::utils::log::entry::kind,return_internal_reference<>())
  .def("content", &TREX::utils::log::entry::content, return_value_policy<copy_const_reference>())
  ;

  // simple log manager
  //   - constructor takes a symbol which will prefix any log messages produced by this class
  //   - name  attribute gives the symbol given at construction
  //   - dir   is a read/write attribute that indicates/sets the log directory
  //   - path  is a read only attribute that gives the trex search path
  //   - add_path adds the path passed as argument to the trex search path
  //   - use_file locates the file passed as argument in trex search path and return its path if found
  //   - info, wran, error produces the string passed as argument as a log message
  class_< log_wrapper, boost::shared_ptr<log_wrapper> >("log", "Logging for trex", init<TREX::utils::Symbol>())
  .def_readonly("name", &log_wrapper::m_name)
  .add_property("dir", &log_wrapper::get_log_dir, &log_wrapper::set_log_dir, "TREX log directory")
  .add_property("path", &log_wrapper::path, "TREX file search path")
  .def("use_file", &log_wrapper::use)
  .def("info", &log_wrapper::info)
  .def("warn", &log_wrapper::warn)
  .def("error", &log_wrapper::error)
  .def("add_path", &log_wrapper::add_path)
  ;
  
  class_< py_log_handler_wrap, boost::noncopyable>("log_handler", "Logging entry handler")
  .add_property("connected", &py_log_handler::connected, "Checks if the handler is still active")
  .def("disconnect", &py_log_handler::disconnect)
  .def("new_entry", pure_virtual(&py_log_handler::new_entry));
  
  // todo : make a basic way to go through XML property tree
  
} // BOOST_PYTHON_MODULE(trex)

void py_log_handler_wrap::new_entry(TREX::utils::log::entry::pointer e) {
  this->get_override("new_entry")(e);
}

