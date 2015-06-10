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
  SingletonUse<LogManager> s_log;

  /** @brief Light reactor declaration */
  TeleoReactor::xml_factory::declare<py_reactor> decl("PyReactor");

//  py_producer python_decl("PyReactor");
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
 * class TREX::python::reactor_proxy
 */

// structors

reactor_proxy::reactor_proxy(py_wrapper const &r)
:m_impl(r.me) {}

reactor_proxy::~reactor_proxy() {}

// observers

Symbol const &reactor_proxy::name() const {
  return m_impl->getName();
}

TICK reactor_proxy::latency() const {
  return m_impl->getLatency();
}

TICK reactor_proxy::lookahead() const {
  return m_impl->getLookAhead();
}

TICK reactor_proxy::exec_latency() const {
  return m_impl->getExecLatency();
}


TICK reactor_proxy::initial() const {
  return m_impl->getInitialTick();
}

TICK reactor_proxy::final() const {
  return m_impl->getFinalTick();
}

TICK reactor_proxy::current() const {
  return m_impl->getCurrentTick();
}

std::string reactor_proxy::date_str(TICK val) const {
  return m_impl->date_str(val);
}

bool reactor_proxy::is_internal(Symbol const &tl) const {
  return m_impl->isInternal(tl);
}

bool reactor_proxy::is_external(Symbol const &tl) const {
  return m_impl->isExternal(tl);
}


// modifiers

void reactor_proxy::use_tl(Symbol const &tl, bool control) {
  m_impl->use(tl, control);
}

void reactor_proxy::provide_tl(Symbol const &tl, bool control) {
  m_impl->provide(tl, control);
}

void reactor_proxy::post(Observation const &o, bool verbose) {
  m_impl->postObservation(o, verbose);
}



/*
 * class TREX::python::reactor_wrap
 */

// structors

reactor_wrap::reactor_wrap(py_wrapper const &r)
:reactor_proxy(r) {
  
}

reactor_wrap::~reactor_wrap() {}

// callbacks

void reactor_wrap::notify(Observation const &o) {
  bp::override f = this->get_override("notify");
  if( f )
    f(o);
  else
    notify_default(o);
}

void reactor_wrap::notify_default(Observation const &o) {
  this->reactor_proxy::notify(o);
}

bool reactor_wrap::synchronize() {
  return this->get_override("synchronize")();
}




/*
 * class TREX::python::py_reactor
 */

// structors

py_reactor::py_reactor(xml_arg_type arg)
:TeleoReactor(arg) {
  boost::property_tree::ptree::value_type &node = xml_factory::node(arg);
  std::string class_name = parse_attr<std::string>(node, "python_class");
  
  syslog()<<"Looking for python class \""<<class_name<<"\"";
  try {
    bp::object my_class = bp::eval(bp::str(class_name));
  
    if( my_class.is_none() ) {
      syslog(log::error)<<"Python class \""<<class_name<<"\" not found.";
      throw XmlError(node, "Python class \""+class_name+"\" not found.");
    }
    syslog()<<"Creating new instance of "<<class_name;
    py_wrapper wrap(this);
    m_self = my_class(wrap);
  } catch(bp::error_already_set const &e) {
    log_error(e);
    throw;
  }
}

py_reactor::~py_reactor() {}


// callbacks

void py_reactor::notify(Observation const &o) {
  m_self.attr("notify")(o);
}


bool py_reactor::synchronize() {
  return m_self.attr("synchronize")();
}




///*
// * class TREX::python::python_producer
// */
//
//py_producer::py_producer(Symbol const &name)
//:TeleoReactor::xml_factory::factory_type::producer(name) {
//  TeleoReactor::xml_factory::factory_type::producer::notify();
//}
//
//py_producer::result_type py_producer::produce(py_producer::argument_type arg) const {
//  // Extract the target python type name
//  boost::property_tree::ptree::value_type &node = TeleoReactor::xml_factory::node(arg);
//  std::string class_name = parse_attr<std::string>(node, "python_class");
//  s_log->syslog("python")<<"Looking for python class "<<class_name;
//  bp::object my_class = bp::eval(bp::str(class_name));
//  
//  if( my_class.is_none() ) {
//    s_log->syslog("python", TREX::utils::log::error)<<"Python class \""<<class_name<<"\" not found";
//    throw XmlError(node, "Python class \""+class_name+"\" not found");
//  }
//  
//  try {
//    s_log->syslog("python")<<"Create "<<class_name<<" instance";
//    bp::object obj = my_class(arg);
//    s_log->syslog("python", TREX::utils::log::info)<<"Created new python object "<<std::string(bp::extract<std::string>(bp::str(obj)));
//    SHARED_PTR<py_wrap> pr = bp::extract< SHARED_PTR<py_wrap> >(obj);
//    SHARED_PTR<TeleoReactor> r(pr);
//    
//    s_log->syslog("python", TREX::utils::log::info)<<"Object is the reactor "<<r->getName();
//    return r;
//  } catch(bp::error_already_set const &e) {
//    log_error(e);
//    throw;
//  } catch(...) {
//    s_log->syslog("python", TREX::utils::log::error)<<"Unknown error while trying to create python reactor of type \""<<class_name<<"\".";
//    throw;
//  }
//}
