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
#include "python_reactor.hh"
#include <trex/utils/Plugin.hh>
#include <trex/python/python_thread.hh>


using namespace TREX::example;
using TREX::transaction::TeleoReactor;
using TREX::python::scoped_gil_release;
using namespace TREX::utils;
namespace bp=boost::python;

namespace {
  SingletonUse<LogManager> s_log;

  TeleoReactor::xml_factory::declare<python_reactor> decl("PythonExample");
}

namespace TREX {
  
  void initPlugin() {
    ::s_log->syslog("plugin.python", log::info)<<"Plugin loaded";
  }

}

python_reactor::python_reactor(TeleoReactor::xml_arg_type arg)
:TeleoReactor(arg, false) {}

python_reactor::~python_reactor() {}

void python_reactor::handleInit() {
  try {
    scoped_gil_release lock;
    bp::import("trex");
    my_var = bp::eval("-1");
  } catch(bp::error_already_set const e) {
    m_exc->unwrap_py_error();
  }
}

bool python_reactor::synchronize() {
  try {
    scoped_gil_release lock;
    my_var = my_var + 1;
  
    bp::str st(my_var);
    bp::str val(my_var.attr("__str__")());
    syslog()<<boost::python::extract<char const *>(val)<<" "
    <<boost::python::extract<char const *>(st);
    return true;
  }catch(bp::error_already_set const e) {
    m_exc->unwrap_py_error();
  }
  return false;
}







