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
#include "python_reactor.hh"

using namespace TREX::python;
using namespace TREX::transaction;
using namespace TREX::utils;

namespace bp=boost::python;

namespace  {
  singleton::use<LogManager> s_log;
  
  producer python_decl("PyReactor");
}

void TREX::python::log_error(bp::error_already_set const &e) {
  PyObject *py_type, *py_val, *py_trace;
  
  PyErr_Fetch(&py_type, &py_val, &py_trace);
  
  std::string msg = bp::extract<std::string>(py_val);
  s_log->syslog("<python>", TREX::utils::log::error)<<msg;
  //Set back error info, display and rethrow
  PyErr_Restore(py_type, py_val, py_trace);
  PyErr_Print();
}

/*
 * class TREX::python::python_reactor
 */

void python_reactor::log_msg(Symbol const &type, std::string const &msg) {
  syslog(type)<<msg;
}

void python_reactor::ext_use(Symbol const &tl, bool control) {
  use(tl, control);
}

bool python_reactor::ext_check(Symbol const &tl) const {
  return isExternal(tl);
}

void python_reactor::post_request(goal_id const &g) {
  postGoal(g);
}

bool python_reactor::cancel_request(goal_id const &g) {
  return postRecall(g);
}

bool python_reactor::ext_unuse(Symbol const &tl) {
  return unuse(tl);
}

void python_reactor::int_decl(Symbol const &tl, bool control) {
  provide(tl, control);
}

bool python_reactor::int_check(Symbol const &tl) const {
  return isInternal(tl);
}

void python_reactor::post_obs(Observation const &obs, bool verb) {
  postObservation(obs, verb);
}

bool python_reactor::int_undecl(Symbol const &tl) {
  return unprovide(tl);
}

// Now all the methods that can be altered

void python_reactor::notify(Observation const &o) {
  try {
    bp::override fn = this->get_override("notify");
    if( fn )
      fn(obs);
    else
      TeleoReactor::notify(o);
  } catch(bp::error_already_set const &e) {
    log_error(e);
    throw;
  }
}

void python_reactor::handleRequest(goal_id const &g) {
  try {
    bp::override fn = this->get_override("handle_request");
    if( fn )
      fn(g);
    else
      TeleoReactor::handleRequest(g);
  } catch(bp::error_already_set const &e) {
    log_error(e);
    throw;
  }
}

void python_reactor::handleRecall(goal_id const &g) {
  try {
    bp::override fn = this->get_override("handle_recall");
    if( fn )
      fn(g);
    else
      TeleoReactor::handleRecall(g);
  } catch(bp::error_already_set const &e) {
    log_error(e);
    throw;
  }
}

// execution callbacks
void python_reactor::handleInit() {
  try {
    bp::override fn = this->get_override("handle_init");
    if( fn )
      fn();
    else
      TeleoReactor::handleInit();
  } catch(bp::error_already_set const &e) {
    log_error(e);
  }
}

void python_reactor::handleTickStart() {
  try {
    bp::override fn = this->get_override("handle_new_tick");
    if( fn )
      fn();
    else
      TeleoReactor::handleTickStart();
  } catch(bp::error_already_set const &e) {
    log_error(e);
  }
}

bool python_reactor::synchronize() {
  try {
    // pure virtual => it has to exist
    bool ret = this->get_override("synchronize")();
    return ret;
  } catch(bp::error_already_set const &e) {
    log_error(e);
    return false;
  }
}

bool python_reactor::hasWork() {
  try {
    bp::override fn = this->get_override("has_work");
    if( fn ) {
      bool ret = fn();
      return ret;
    } else
      return TeleoReactor::hasWork(); // false
  } catch(bp::error_already_set const &e) {
    log_error(e);
    return false;
  }
}

void python_reactor::resume() {
  try {
    bp::override fn = this->get_override("resume");
    if( fn )
      fn();
    else
      TeleoReactor::resume();
  } catch(bp::error_already_set const &e) {
    log_error(e);
  }
}

/*
 * class TREX::python::python_producer
 */

producer::producer(Symbol const &name)
:TeleoReactor::xml_factory::factory_type::producer(name) {
  TeleoReactor::xml_factory::factory_type::producer::notify();
}

producer::result_type producer::produce(producer::argument_type arg) const {
  // Extract the target python type name
  boost::property_tree::ptree::value_type &node = TeleoReactor::xml_factory::node(arg);
  std::string class_name = parse_attr<std::string>(node, "python_class");
  bp::object my_class = bp::eval(bp::str(class_name));
    
  if( my_class.is_none() ) {
    s_log->syslog("python", TREX::utils::log::error)<<"Python class \""<<class_name<<"\" not found";
    throw XmlError(node, "Python class \""+class_name+"\" not found");
  }
  
  try {
    bp::object obj = my_class(arg);
    s_log->syslog("python", TREX::utils::log::info)<<"Created new python object "<<std::string(bp::extract<std::string>(bp::str(obj)));
    result_type r = bp::extract<result_type>(obj);
    
    s_log->syslog("python", TREX::utils::log::info)<<"Object is the reactor "<<r->getName();
    return r;
  } catch(bp::error_already_set const &e) {
    log_error(e);
    throw;
  } catch(...) {
    s_log->syslog("python", TREX::utils::log::error)<<"Unknown error while trying to create python reactor of type \""<<class_name<<"\".";
    throw;
  }
}
