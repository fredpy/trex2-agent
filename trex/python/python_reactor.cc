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
#include "python_thread.hh"

using namespace TREX::python;
using namespace TREX::transaction;
using namespace TREX::utils;

namespace bp=boost::python;

namespace  {
  /** @brief Light reactor declaration */
  TeleoReactor::xml_factory::declare<py_reactor> decl("PyReactor");
}

/*
 * class TREX::python::reactor_proxy
 */

// structors

reactor_proxy::reactor_proxy(py_wrapper const &r)
:m_impl(r.me), m_active(true) {
  m_impl->syslog()<<"Proxy created with address: "<<this;
}

reactor_proxy::~reactor_proxy() {}

void reactor_proxy::disconnect() {
  m_active = false;
}


// observers

bool reactor_proxy::is_verbose() const {
  return m_impl->is_verbose();
}

Symbol const &reactor_proxy::name() const {
  return m_impl->getName();
}

Symbol const &reactor_proxy::agent_name() const {
  return m_impl->getAgentName();
}

graph const &reactor_proxy::graph() const {
  return m_impl->getGraph();
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

double reactor_proxy::tick_duration() const {
  typedef CHRONO::duration<double> fl_secs;
  return CHRONO::duration_cast<fl_secs>(m_impl->tickDuration()).count();
}

double reactor_proxy::as_seconds(TICK delta) const {
  typedef CHRONO::duration<double> fl_secs;
  return CHRONO::duration_cast<fl_secs>(m_impl->tickDuration()*delta).count();
}


// modifiers



void reactor_proxy::set_verbose(bool value) {
  if( value )
    m_impl->set_verbose();
  else
    m_impl->reset_verbose();
}


void reactor_proxy::set_latency(TICK val) {
  if( m_active )
    m_impl->update_latency(val);
}

void reactor_proxy::set_lookahead(TICK val) {
  if( m_active )
    m_impl->update_horizon(val);
}

void reactor_proxy::use_tl(Symbol const &tl, bool control,
                           bool plan_listen) {
  if( m_active )
    m_impl->use(tl, control, plan_listen);
}

bool reactor_proxy::unuse_tl(utils::Symbol const &tl) {
  return m_active && m_impl->unuse(tl);
}

void reactor_proxy::provide_tl(Symbol const &tl, bool control,
                               bool plan_publish) {
  if( m_active )
    m_impl->provide(tl, control, plan_publish);
}

bool reactor_proxy::unprovide_tl(utils::Symbol const &tl) {
  return m_active && m_impl->unprovide(tl);
}

void reactor_proxy::post(Observation const &o, bool verbose) {
  if( m_active )
    m_impl->postObservation(o, verbose);
}

bool reactor_proxy::request(goal_id const &g) {
  return m_active && m_impl->postGoal(g);
}

bool reactor_proxy::recall(goal_id const &g) {
  return m_active && m_impl->postRecall(g);
}

bool reactor_proxy::post_plan(goal_id const &g) {
  return m_active && m_impl->postPlanToken(g);
}

void reactor_proxy::cancel_plan(goal_id const &g) {
  if( m_active )
    m_impl->cancelPlanToken(g);
}

void reactor_proxy::info(std::string const &msg) {
  m_impl->syslog(log::info)<<msg;
}

void reactor_proxy::warning(std::string const &msg) {
  m_impl->syslog(log::warn)<<msg;
}

void reactor_proxy::error(std::string const &msg) {
  m_impl->syslog(log::error)<<msg;
}


// callbacks

bool reactor_proxy::has_work() {
  return false;
}


/*
 * class TREX::python::reactor_wrap
 */

// structors

reactor_wrap::reactor_wrap(py_wrapper const &r)
:reactor_proxy(r) {
  
}

reactor_wrap::~reactor_wrap() {
}

// callbacks

void reactor_wrap::terminate() {
  bp::override f = this->get_override("__terminate__");
  if( f )
    f();
  disconnect();
}

void reactor_wrap::handle_init() {
  bp::override f = this->get_override("handle_init");
  if( f )
    f();
  else
    handle_init_default();
}

void reactor_wrap::handle_init_default() {
  this->reactor_proxy::handle_init();
}

void reactor_wrap::handle_request(goal_id const &g) {
  bp::override f = this->get_override("handle_request");
  if( f ) {
    f(bp::ptr(g.get()));
  } else
    handle_request_default(g);
}

void reactor_wrap::handle_request_default(goal_id const &g) {
  this->reactor_proxy::handle_request(g);
}

void reactor_wrap::handle_recall(goal_id const &g) {
  bp::override f = this->get_override("handle_recall");
  if( f )
    f(bp::ptr(g.get()));
  else
    handle_recall_default(g);
}

void reactor_wrap::handle_recall_default(goal_id const &g) {
  this->reactor_proxy::handle_recall(g);
}


void reactor_wrap::handle_new_tick() {
  bp::override f = this->get_override("handle_new_tick");
  if( f )
    f();
  else
    handle_new_tick_default();
}

void reactor_wrap::handle_new_tick_default() {
  this->reactor_proxy::handle_new_tick();
}

void reactor_wrap::notify(Observation const &o) {
  bp::override f = this->get_override("notify");
  if( f )
    f(boost::ref(o));
  else
    notify_default(o);
}

void reactor_wrap::notify_default(Observation const &o) {
  this->reactor_proxy::notify(o);
}

bool reactor_wrap::synchronize() {
  bp::override sync = this->get_override("synchronize");

  if( sync )
    return sync();
  else
    throw std::runtime_error("Failed to find override function for synchronize");
}

bool reactor_wrap::has_work() {
  bp::override f = this->get_override("has_work");
  if( f )
    return f();
  else
    return has_work_default();
}

bool reactor_wrap::has_work_default() {
  return this->reactor_proxy::has_work();
}

void reactor_wrap::resume() {
  bp::override f = this->get_override("resume");
  if( f )
    f();
  else
    resume_default();
}

void reactor_wrap::resume_default() {
  this->reactor_proxy::resume();
}

void reactor_wrap::new_plan(goal_id const &g) {
  bp::override f = this->get_override("new_plan");
  if( f )
    f(bp::ptr(g.get()));
  else
    new_plan_default(g);
}

void reactor_wrap::new_plan_default(goal_id const &g) {
  this->reactor_proxy::new_plan(g);
}

void reactor_wrap::cancelled_plan(goal_id const &g) {
  bp::override f = this->get_override("cancelled_plan");
  if( f )
    f(bp::ptr(g.get()));
  else
    cancelled_plan_default(g);
}


void reactor_wrap::cancelled_plan_default(goal_id const &g) {
  this->reactor_proxy::cancelled_plan(g);
}



/*
 * class TREX::python::py_reactor
 */

// structors

py_reactor::py_reactor(xml_arg_type arg)
:TeleoReactor(arg, false, true) {
  try {
    boost::property_tree::ptree::value_type &node = xml_factory::node(arg);
    std::string class_name = parse_attr<std::string>(node, "python_class");
    boost::optional<std::string> source = parse_attr< boost::optional<std::string> >(node, "file");
    
    bp::object scope_d;

    if( source ) {
      bool found;
      std::string py = manager().use(*source, found);
      
      if( !found ) {
        py = manager().use((*source)+".py", found);
        if( !found )
          throw XmlError(node, "Unable to locate "+(*source)+"[.py]");
      }
      syslog()<<"exec python script: "<<py;
      scoped_gil_release lock;
      m_scope = m_python->add_module(m_python->main(), "_trex");
      m_scope = m_python->add_module(m_scope, getAgentName().str());
      m_scope = m_python->add_module(m_scope, getName().str());
      
      
      bp::scope my_scope = m_scope;
      scope_d = m_scope.attr("__dict__");
      scope_d["__builtins__"] = m_python->main_env()["__builtins__"];
      bp::str py_name(py);
      
      bp::exec_file(py_name, scope_d);
      
//      std::string dir = bp::extract<std::string>(bp::str(m_python->dir(m_scope)));
//      syslog()<<"env: "<<dir;
//      dir = bp::extract<std::string>(bp::str(m_python->dir(m_python->main())));
//      syslog()<<"main: "<<dir;
    }
    
    
    syslog()<<"Looking for python class \""<<class_name<<"\"";
    scoped_gil_release lock;
    
    bp::object my_class = bp::eval(bp::str(class_name), scope_d);

    if( my_class.is_none() ) {
      syslog(log::error)<<"Python class \""<<class_name<<"\" not found.";
      throw XmlError(node, "Python class \""+class_name+"\" not found.");
    }
    
    syslog()<<"Creating new instance of "<<class_name;
    py_wrapper wrap(this, node);
    m_obj = my_class(wrap);
    
    bp::extract<reactor_wrap &> extractor(m_obj);
    if( !extractor.check() ) {
      syslog(log::error)<<"Python class "<<class_name<<" is not a reactor";
      throw ReactorException(*this, "Python class "+class_name+
                             " is not a reactor");
    }
  } catch(bp::error_already_set const &e) {
    m_exc->unwrap_py_error();
  }
}

py_reactor::~py_reactor() {
  if( !m_obj.is_none() ) {
    syslog()<<"Destroying obj";
    self().terminate();
  }
}


// observers

reactor_proxy &py_reactor::self() {
  return bp::extract<reactor_wrap &>(m_obj);
}


// callbacks

void py_reactor::handleInit() {
  syslog()<<" init";
  try {
    self().handle_init();
  } catch(bp::error_already_set const &e) {
    m_exc->unwrap_py_error();
//    unpack_error("handle_init", e);
  }
}

void py_reactor::handleRequest(goal_id const &g) {
  try {
    self().handle_request(g);
  } catch(bp::error_already_set const &e) {
    m_exc->unwrap_py_error();
//    unpack_error("handle_request", e, false);
  }
}

void py_reactor::handleRecall(goal_id const &g) {
  try {
    self().handle_recall(g);
  } catch(bp::error_already_set const &e) {
    m_exc->unwrap_py_error();
//    unpack_error("handle_recall", e, false);
  }
}

void py_reactor::handleTickStart() {
  try {
    self().handle_new_tick();
  } catch(bp::error_already_set const &e) {
    m_exc->unwrap_py_error();
//    unpack_error("handle_new_tick", e, false);
  }
}


void py_reactor::notify(Observation const &o) {
  try {
    self().notify(o);
  } catch(bp::error_already_set const &e) {
    m_exc->unwrap_py_error();
//    unpack_error("notify", e, false);
  }
}


bool py_reactor::synchronize() {
  try {
    return self().synchronize();
  } catch(bp::error_already_set const &e) {
    m_exc->unwrap_py_error();
//    unpack_error("synchronize", e);
    return false;
  }
}

bool py_reactor::hasWork() {
  try {
    return self().has_work();
  } catch(bp::error_already_set const &e) {
    m_exc->unwrap_py_error();
//    unpack_error("has_work", e, false);
    return false;
  }
}

void py_reactor::resume() {
  try {
    self().resume();
  } catch(bp::error_already_set const &e) {
    m_exc->unwrap_py_error();
//    unpack_error("resume", e, false);
  }
}

void py_reactor::newPlanToken(goal_id const &g) {
  try {
    self().new_plan(g);
  } catch(bp::error_already_set const &e) {
    m_exc->unwrap_py_error();
//    unpack_error("new_plan", e, false);
  }
}

void py_reactor::cancelledPlanToken(goal_id const &g) {
  try {
    self().cancelled_plan(g);
  } catch(bp::error_already_set const &e) {
    m_exc->unwrap_py_error();
//    unpack_error("cancelled_plan", e, false);
  }
}

