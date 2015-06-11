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
  m_impl->update_latency(val);
}

void reactor_proxy::set_lookahead(TICK val) {
  m_impl->update_horizon(val);
}

void reactor_proxy::use_tl(Symbol const &tl, bool control,
                           bool plan_listen) {
  m_impl->use(tl, control, plan_listen);
}

bool reactor_proxy::unuse_tl(utils::Symbol const &tl) {
  return m_impl->unuse(tl);
}

void reactor_proxy::provide_tl(Symbol const &tl, bool control,
                               bool plan_publish) {
  m_impl->provide(tl, control, plan_publish);
}

bool reactor_proxy::unprovide_tl(utils::Symbol const &tl) {
  return m_impl->unprovide(tl);
}

void reactor_proxy::post(Observation const &o, bool verbose) {
  m_impl->postObservation(o, verbose);
}

bool reactor_proxy::request(goal_id const &g) {
  return m_impl->postGoal(g);
}

bool reactor_proxy::recall(goal_id const &g) {
  return m_impl->postRecall(g);
}

bool reactor_proxy::post_plan(goal_id const &g) {
  return m_impl->postPlanToken(g);
}

void reactor_proxy::cancel_plan(goal_id const &g) {
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

reactor_wrap::~reactor_wrap() {}

// callbacks

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
  if( f )
    f(g);
  else
    handle_request_default(g);
}

void reactor_wrap::handle_request_default(goal_id const &g) {
  this->reactor_proxy::handle_request(g);
}

void reactor_wrap::handle_recall(goal_id const &g) {
  bp::override f = this->get_override("handle_recall");
  if( f )
    f(g);
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
    f(g);
  else
    new_plan_default(g);
}

void reactor_wrap::new_plan_default(goal_id const &g) {
  this->reactor_proxy::new_plan(g);
}

void reactor_wrap::cancelled_plan(goal_id const &g) {
  bp::override f = this->get_override("cancelled_plan");
  if( f )
    f(g);
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
:TeleoReactor(arg, false, false) {
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

void py_reactor::handleInit() {
  m_self.attr("handle_init")();
}

void py_reactor::handleRequest(goal_id const &g) {
  m_self.attr("handle_request")(g);
}

void py_reactor::handleRecall(goal_id const &g) {
  m_self.attr("handle_recall")(g);  
}

void py_reactor::handleTickStart() {
  m_self.attr("handle_new_tick")();
}


void py_reactor::notify(Observation const &o) {
  m_self.attr("notify")(o);
}


bool py_reactor::synchronize() {
  return m_self.attr("synchronize")();
}

bool py_reactor::hasWork() {
  return m_self.attr("has_work")();
}

void py_reactor::resume() {
  m_self.attr("resume")();
}

void py_reactor::newPlanToken(goal_id const &g) {
  m_self.attr("new_plan")(g);
}

void py_reactor::cancelledPlanToken(goal_id const &g) {
  m_self.attr("cancelled_plan")(g);
}

