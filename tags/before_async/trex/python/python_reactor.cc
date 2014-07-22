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
  singleton::use<log_manager> s_log;
  
//  producer python_decl("PyReactor");
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

void python_reactor::log_msg(symbol const &type, std::string const &msg) {
  syslog(type)<<msg;
}

void python_reactor::ext_use(symbol const &tl, bool control) {
  use(tl, control);
}

bool python_reactor::ext_check(symbol const &tl) const {
  return is_external(tl);
}

void python_reactor::post_request(token_id const &g) {
  post_goal(g);
}

bool python_reactor::cancel_request(token_id const &g) {
  return post_recall(g);
}

bool python_reactor::ext_unuse(symbol const &tl) {
  return unuse(tl);
}

void python_reactor::int_decl(symbol const &tl, bool control) {
  provide(tl, control);
}

bool python_reactor::int_check(symbol const &tl) const {
  return is_internal(tl);
}

void python_reactor::post_obs(token const &obs, bool verb) {
  post_observation(obs, verb);
}

bool python_reactor::int_undecl(symbol const &tl) {
  return unprovide(tl);
}

// Now all the methods that can be altered

void python_reactor::notify(token const &o) {
  try {
    bp::override fn = this->get_override("notify");
    if( fn )
      fn(obs);
    else
      reactor::notify(o);
  } catch(bp::error_already_set const &e) {
    log_error(e);
    throw;
  }
}

void python_reactor::handle_request(token_id const &g) {
  try {
    bp::override fn = this->get_override("handle_request");
    if( fn )
      fn(g);
    else
      reactor::handle_request(g);
  } catch(bp::error_already_set const &e) {
    log_error(e);
    throw;
  }
}

void python_reactor::handle_recall(token_id const &g) {
  try {
    bp::override fn = this->get_override("handle_recall");
    if( fn )
      fn(g);
    else
      reactor::handle_recall(g);
  } catch(bp::error_already_set const &e) {
    log_error(e);
    throw;
  }
}

// execution callbacks
void python_reactor::handle_init() {
  try {
    bp::override fn = this->get_override("handle_init");
    if( fn )
      fn();
    else
      reactor::handle_init();
  } catch(bp::error_already_set const &e) {
    log_error(e);
  }
}

void python_reactor::handle_tick_start() {
  try {
    bp::override fn = this->get_override("handle_new_tick");
    if( fn )
      fn();
    else
      reactor::handle_tick_start();
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

bool python_reactor::has_work() {
  try {
    bp::override fn = this->get_override("has_work");
    if( fn ) {
      bool ret = fn();
      return ret;
    } else
      return reactor::has_work(); // false
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
      reactor::resume();
  } catch(bp::error_already_set const &e) {
    log_error(e);
  }
}

/*
 * class TREX::python::python_producer
 */

//producer::producer(symbol const &name)
//:reactor::factory::factory_type::producer(name) {
//  reactor::factory::factory_type::producer::notify();
//}
//
//producer::result_type producer::produce(producer::argument_type arg) const {
//  // Extract the target python type name
//  boost::property_tree::ptree::value_type &node = reactor::factory::node(arg);
//  std::string class_name = parse_attr<std::string>(node, "python_class");
//  bp::object my_class = bp::eval(bp::str(class_name));
//    
//  if( my_class.is_none() ) {
//    s_log->syslog("python", TREX::utils::log::error)<<"Python class \""<<class_name<<"\" not found";
//    throw boost::property_tree::ptree_bad_data("Python class \""+class_name+"\" not found", node);
//  }
//  
//  try {
//    bp::object obj = my_class(arg);
//    s_log->syslog("python", TREX::utils::log::info)<<"Created new python object "<<std::string(bp::extract<std::string>(bp::str(obj)));
//    result_type r = bp::extract<result_type>(obj);
//    
//    s_log->syslog("python", TREX::utils::log::info)<<"Object is the reactor "<<r->name();
//    return r;
//  } catch(bp::error_already_set const &e) {
//    log_error(e);
//    throw;
//  } catch(...) {
//    s_log->syslog("python", TREX::utils::log::error)<<"Unknown error while trying to create python reactor of type \""<<class_name<<"\".";
//    throw;
//  }
//}
