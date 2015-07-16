/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Frederic Py.
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
#include <boost/python.hpp>
#include "python_env.hh"
#include "python_thread.hh"
#include <iostream>

using namespace TREX::python;
namespace bp=boost::python;
namespace tlog=TREX::utils::log;

/*
 * class TREX::python::python_env
 */

python_env::python_env() {
  static char name[] = {'\0'};
  static char *argv[] = { name };
  m_log->syslog(tlog::info)<<"Initializing python";
  Py_Initialize();
  // Needed for some python libs inclufing rospy
  PySys_SetArgv(1, argv);
}

python_env::~python_env() {
}

bp::object &python_env::import(std::string const &module) {
  scoped_gil_release lock;
  object_map::iterator i = m_loaded.find(module);
  
  if( m_loaded.end()==i ) {
    m_log->syslog(tlog::info)<<"Loading python module \""<<module<<"\"";
    bp::object mod = bp::import(module.c_str());
    i = m_loaded.insert(object_map::value_type(module, mod)).first;
  }
  return i->second;
}

bp::object python_env::load_module_for(std::string const &type) {
  size_t pos = type.find_last_of('.');
  if( pos!=std::string::npos ) {
    std::string loaded = type.substr(0, pos);
    bp::object tmp = import(loaded);
    return tmp.attr(type.substr(pos+1).c_str());
  }
  return bp::object();
}


bp::object python_env::dir(bp::object const &obj) const {
  bp::handle<> ret(PyObject_Dir(obj.ptr()));
  return bp::object(ret);  
}

bool python_env::callable(bp::object const &obj) const {
  return 1==PyCallable_Check(obj.ptr());
}

