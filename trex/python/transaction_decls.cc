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
#include <trex/transaction/TeleoReactor.hh>

#include <boost/python.hpp>

using namespace boost::python;

namespace tu=TREX::utils;
namespace tt=TREX::transaction;

namespace {
  
  
  class PyReactor :public tt::TeleoReactor {
  public:
    void log_msg(tu::Symbol const &type, std::string const &msg) {
      syslog(type)<<msg;
    }
    void info(std::string const &msg) {
      log_msg(tu::log::info, msg);
    }
    void warn(std::string const &msg) {
      log_msg(tu::log::warn, msg);
    }
    void error(std::string const &msg) {
      log_msg(tu::log::error, msg);
    }
    
    void ext_use(tu::Symbol const &tl, bool control) {
      use(tl, control);
    }
    bool ext_unuse(tu::Symbol const &tl) {
      return unuse(tl);
    }
    bool ext_check(tu::Symbol const &tl) const {
      return isExternal(tl);
    }
    
    void int_decl(tu::Symbol const &tl, bool control) {
      provide(tl, control);
    }
    bool int_undecl(tu::Symbol const &tl) {
      return unprovide(tl);
    }
    bool int_check(tu::Symbol const &tl) const {
      return isInternal(tl);
    }
    
    void post_obs(tt::Observation const &obs, bool verb) {
      postObservation(obs, verb);
    }
    
    void post_request(tt::goal_id const &g) {
      postGoal(g);
    }
    bool cancel_request(tt::goal_id const &g) {
      return postRecall(g);
    }
    
  protected:
    explicit PyReactor(tt::TeleoReactor::xml_arg_type &arg)
    :tt::TeleoReactor(arg) {}
    virtual ~PyReactor() {}
    
  };
  
  class py_decl: public tt::TeleoReactor::xml_factory::factory_type::producer {
  public:
    py_decl(tu::Symbol const &name)
    :tt::TeleoReactor::xml_factory::factory_type::producer(name) {
      tt::TeleoReactor::xml_factory::factory_type::producer::notify();
    }
    ~py_decl() {}
    
  private:
    result_type produce(argument_type arg) const {
      // I am not sure of the validity and even less robustness of this code ....
      
      boost::property_tree::ptree::value_type &node = tt::TeleoReactor::xml_factory::node(arg);
      std::string py_class = tu::parse_attr<std::string>(node, "class");
      
      
      object my_class = eval(str(py_class));
    
      // Is my_class valid and derived from PyReactor
      if( my_class.is_none() )
        throw tu::XmlError(node, "python object \""+py_class+"\" not found");
      
      object obj = my_class(arg);
      return extract<result_type>(obj);
    }
    
  };
  
  // declare my python reactor factory
  py_decl _py_decl("PyReactor");
  
  
  struct reactor_wrap: PyReactor, wrapper<PyReactor> {
    explicit reactor_wrap(tt::TeleoReactor::xml_arg_type &arg)
    :PyReactor(arg) {}
    
    
    void notify(tt::Observation const &obs) {
      override fn = this->get_override("notify");
      if( fn )
        fn(obs);
      else
        PyReactor::notify(obs);
    }
    virtual void handleRequest(tt::goal_id const &g) {
      override fn = this->get_override("handle_request");
      if( fn )
        fn(g);
      else
        PyReactor::handleRequest(g);
    }
    virtual void handleRecall(tt::goal_id const &g) {
      override fn = this->get_override("handle_recall");
      if( fn )
        fn(g);
      else
        PyReactor::handleRecall(g);
    }
    
    void handleInit() {
      override fn = this->get_override("handle_init");
      if( fn )
        fn();
      else
        PyReactor::handleInit();
    }

    void handleTickStart() {
      override fn = this->get_override("handle_new_tick");
      if( fn )
        fn();
      else
        PyReactor::handleTickStart();
    }
    
    bool synchronize() {
      return this->get_override("synchronize")();
    }
    
    bool hasWork() {
      override fn = this->get_override("has_work");
      if( fn )
        return fn();
      else
        return PyReactor::hasWork();
    }
    void resume() {
      override fn = this->get_override("resume");
      if( fn )
        fn();
      else
        PyReactor::resume();
    }
    
    
  };
  
  template<class Obj>
  std::string xml_str(Obj const &dom) {
    std::ostringstream oss;
    dom.toXml(oss);
    return oss.str();
  }
  
  template<class Obj>
  std::string str_impl(Obj const &dom) {
    std::ostringstream oss;
    oss<<dom;
    return oss.str();
  }
  
  bool test_id(tt::graph::reactor_id const &id) {
    return id;
  }
  tt::TeleoReactor const &reactor_id(tt::graph::reactor_id const &id) {
    return *id;
  }
  std::string str_id(tt::graph::reactor_id const &id) {
    if( id )
      return id->getAgentName().str()+"."+id->getName().str();
    return "";
  }
  
  

}

void export_transactions() {
  // Setup my submodule
  object module(handle<>(borrowed(PyImport_AddModule("trex.transaction"))));
  scope().attr("transaction") = module;
  scope my_scope = module;
  // from now on everything is under trex.transaction
  
  
  void (tt::Predicate::* attr_1)(tt::Variable const &) = &tt::Predicate::restrictAttribute;
  void (tt::Predicate::* attr_2)(tu::Symbol const &,tt::DomainBase const &) = &tt::Predicate::restrictAttribute;
    

  // TODO: need to give an iterator
  class_<tt::Predicate, boost::noncopyable>("predicate", "trex timeline predicate", no_init)
  .def("object", &tt::Predicate::object, return_internal_reference<>())
  .def("predicate", &tt::Predicate::predicate, return_internal_reference<>())
  .def("has_attribute", &tt::Predicate::hasAttribute, arg("name"))
  .def("restrict", attr_1, arg("var"))
  .def("restrict", attr_2, (arg("name"), arg("domain")))
  .def("attribute", &tt::Predicate::getAttribute, return_internal_reference<>(), arg("name"))
  .def("xml", &xml_str<tt::Predicate>)
  .def("__str__", &str_impl<tt::Predicate>)
  ;
  
  class_<tt::Observation, tt::observation_id, bases<tt::Predicate> >("obs", "trex observation", init<tt::Predicate const &>())
  .def(init<tu::Symbol, tu::Symbol>(args("timeline", "pred"), "Create new observation pred on timeline"))
  ;
                                                                     
  class_<tt::Goal, tt::goal_id, bases<tt::Predicate> >("goal", "trex goal",
                                                       init<tu::Symbol, tu::Symbol>(args("timeline", "pred"), "Create new goal pred on timeline"))
  ;
  
  class_<tt::TICK, boost::noncopyable>("tick", "trex tick date", init<>())
  .def(init<long>())
  .def("__str__", &str_impl<tt::TICK>)
  .def(self == self)
  .def(self != self)
  .def(self < self)
  .def(self <= self)
  .def(self > self)
  .def(self >= self)
  ;
  
  implicitly_convertible<long, tt::TICK>();
  
  
  class_<tt::graph, boost::noncopyable> c_graph("graph", "reactors transaction graph", no_init);
  
  
  
  c_graph.def("name", &tt::graph::getName, return_internal_reference<>())
  .def("empty", &tt::graph::empty)
  .add_property("empty", &tt::graph::empty)
  .add_property("reactors_count", &tt::graph::count_reactors)
  .add_property("relations_count", &tt::graph::count_relations)
  .add_property("current_tick", &tt::graph::getCurrentTick, "current tick date")
  .def("date_str", &tt::graph::date_str, "convert a tick into a date string")
  ;

  class_<tt::TeleoReactor::xml_arg_type>("r_init", "type of argument used ofr a reactor init", no_init)
  .def_readonly("graph", &tt::TeleoReactor::xml_arg_type::second)
  .def_readonly("xml", &tt::TeleoReactor::xml_arg_type::first)
  ;
  
  class_<tt::TeleoReactor, boost::noncopyable>("reactor", "Tha abstract interface for a reactor", no_init)
  .def("name", &tt::TeleoReactor::getName, return_internal_reference<>())
  .def("agent_name", &tt::TeleoReactor::getAgentName, return_internal_reference<>())
  .def("graph", &tt::TeleoReactor::getGraph, return_internal_reference<>())
  .add_property("initial_tick", &tt::TeleoReactor::getInitialTick)
  .add_property("current_tick", &tt::TeleoReactor::getCurrentTick)
  .add_property("final_tick", &tt::TeleoReactor::getFinalTick)
  .def("date_str", &tt::TeleoReactor::date_str, "convert a tick into a date string")
  .add_property("latency", &tt::TeleoReactor::getLatency)
  .add_property("look_ahead", &tt::TeleoReactor::getLookAhead)
  .add_property("exec_latency", &tt::TeleoReactor::getExecLatency)
  .add_property("externals_count", &tt::TeleoReactor::count_externals)
  .add_property("internals_count", &tt::TeleoReactor::count_internals)
  .add_property("work_ratio", &tt::TeleoReactor::workRatio)
  .def("__str__", &str_id)
  ;
  
  c_graph.def("add_reactor", static_cast<tt::graph::reactor_id (tt::graph::*)(boost::property_tree::ptree::value_type &)>(&tt::graph::add_reactor), return_internal_reference<>())
  .def("add_reactors", static_cast<size_t (tt::graph::*)(boost::property_tree::ptree &)>(&tt::graph::add_reactors))
  .def("kill_reactor", &tt::graph::kill_reactor)
  ;
  
  class_<reactor_wrap, bases<tt::TeleoReactor>, boost::noncopyable>("py_reactor", "Base class for python based reactors", init<tt::TeleoReactor::xml_arg_type &>())
  // transaction callbacks
//  .def("notify", &tt::TeleoReactor::notify, arg("o"))
//  .def("handle_request", &reactor_wrap::handleRequest, arg("g"))
//  .def("handle_recall", &reactor_wrap::handleRequest, arg("g"))
  // execution callbacks
  .def("handle_init", &reactor_wrap::handleInit)
  .def("handle_new_tick", &reactor_wrap::handleTickStart)
  .def("synchronize", pure_virtual(&reactor_wrap::synchronize))
  .def("has_work", &reactor_wrap::hasWork)
  .def("resume", &reactor_wrap::resume)
  // transaction calls
  .def("is_external", &PyReactor::ext_check, arg("tl_name"))
  .def("use", &PyReactor::ext_use, (arg("tl_name"), arg("controled")=true))
  .def("unuse", &PyReactor::ext_unuse, arg("tl_name"))
  .def("is_internal", &PyReactor::int_check, arg("tl_name"))
  .def("use", &PyReactor::int_decl, (arg("tl_name"), arg("controlable")=true))
  .def("unuse", &PyReactor::int_undecl, arg("tl_name"))
  .def("post", &PyReactor::post_obs, (arg("o"), arg("verbose")=false))
  .def("request", &PyReactor::post_request, arg("g"))
  .def("recall", &PyReactor::cancel_request, arg("g"))
  // logging
  .def("info", &PyReactor::info, arg("msg"))
  .def("warn", &PyReactor::warn, arg("msg"))
  .def("error", &PyReactor::error, arg("msg"))
  ;
  
  
}