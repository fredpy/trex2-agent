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
  

  void log_error(error_already_set const &e) {
    PyObject *py_type, *py_val, *py_trace;
    tu::SingletonUse<tu::LogManager> log;

    PyErr_Fetch(&py_type, &py_val, &py_trace);
    
    std::string msg = extract<std::string>(py_val);
    log->syslog("python", tu::log::error)<<msg;
    //Set back error info, display and rethrow
    PyErr_Restore(py_type, py_val, py_trace);
    PyErr_Print();
    throw e;
  }
  
  
  struct python_reactor: tt::TeleoReactor, wrapper<tt::TeleoReactor> {

    // Factory constructor
    explicit python_reactor(tt::TeleoReactor::xml_arg_type &arg)
    :tt::TeleoReactor(arg) {
      syslog("DEBUG")<<"New python reactor created: \""<<getName()<<"\".";
    }
    virtual ~python_reactor() {
      syslog("DEBUG")<<"Destroying reactor \""<<getName()<<"\".";
    }

    // Logging
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
    
    // timeline management
    void ext_use(tu::Symbol const &tl, bool control) {
      use(tl, control);
    }
    bool ext_check(tu::Symbol const &tl) const {
      return isExternal(tl);
    }
    void post_request(tt::goal_id const &g) {
      postGoal(g);
    }
    bool cancel_request(tt::goal_id const &g) {
      return postRecall(g);
    }
    bool ext_unuse(tu::Symbol const &tl) {
      return unuse(tl);
    }
    

    void int_decl(tu::Symbol const &tl, bool control) {
      provide(tl, control);
    }
    bool int_check(tu::Symbol const &tl) const {
      return isInternal(tl);
    }
    void post_obs(tt::Observation const &obs, bool verb) {
      postObservation(obs, verb);
    }
    bool int_undecl(tu::Symbol const &tl) {
      return unprovide(tl);
    }
    
    // transaction callbacks
    void notify(tt::Observation const &o) {
      try {
        override fn = this->get_override("notify");
        if( fn )
          fn(obs);
        else
          tt::TeleoReactor::notify(o);
      } catch(error_already_set const &e) {
        log_error(e);
      }
    }
    void handleRequest(tt::goal_id const &g) {
      try {
        override fn = this->get_override("handle_request");
        if( fn )
          fn(g);
        else
          tt::TeleoReactor::handleRequest(g);
      } catch(error_already_set const &e) {
        log_error(e);
      }
    }
    void handleRecall(tt::goal_id const &g) {
      try {
        override fn = this->get_override("handle_recall");
        if( fn )
          fn(g);
        else
          tt::TeleoReactor::handleRecall(g);
      } catch(error_already_set const &e) {
        log_error(e);
      }
    }
    
    // execution callbacks
    void handleInit() {
      try {
        override fn = this->get_override("handle_init");
        if( fn )
          fn();
        else
          tt::TeleoReactor::handleInit();
      } catch(error_already_set const &e) {
        log_error(e);
      }
   }
    void handleTickStart() {
      try {
        override fn = this->get_override("handle_new_tick");
        if( fn )
          fn();
        else
          tt::TeleoReactor::handleTickStart();
      } catch(error_already_set const &e) {
        log_error(e);
      }
    }
    bool synchronize() {
      try {
        // pure virtual => it has to exist
        bool ret = this->get_override("synchronize")();
        return ret;
      } catch(error_already_set const &e) {
        log_error(e);
        return false;
      }
    }
    bool hasWork() {
      try {
        override fn = this->get_override("has_work");
        if( fn ) {
          bool ret = fn();
          return ret;
        } else
          return tt::TeleoReactor::hasWork(); // false
      } catch(error_already_set const &e) {
        log_error(e);
        return false;
      }
    }
    void resume() {
      try {
        override fn = this->get_override("resume");
        if( fn )
          fn();
        else
          tt::TeleoReactor::resume();
      } catch(error_already_set const &e) {
        log_error(e);
      }
   }
    
  }; // python_reactor
 
  class python_producer: public tt::TeleoReactor::xml_factory::factory_type::producer {
  public:
    explicit python_producer(tu::Symbol const &name)
    :tt::TeleoReactor::xml_factory::factory_type::producer(name) {
      tt::TeleoReactor::xml_factory::factory_type::producer::notify();
    }
    ~python_producer() {}

  private:
    result_type produce(argument_type arg) const {
      // Extract the target python type name
      boost::property_tree::ptree::value_type &node = tt::TeleoReactor::xml_factory::node(arg);
      std::string class_name = tu::parse_attr<std::string>(node, "python_class");
      object my_class = eval(str(class_name));
      
      if( my_class.is_none() ) {
        m_log->syslog("python", tu::log::error)<<"Python class \""<<class_name<<"\" not found";
        throw tu::XmlError(node, "Python class \""+class_name+"\" not found");
      }
      
      try {
        object obj = my_class(arg);
        m_log->syslog("python", tu::log::info)<<"Created new python object "<<std::string(extract<std::string>(str(obj)));
        boost::shared_ptr<tt::TeleoReactor> r = extract< boost::shared_ptr<tt::TeleoReactor> >(obj);
        m_log->syslog("python", tu::log::info)<<"Object is the reactor "<<r->getName();
        return r;
      } catch(error_already_set const &e) {
        log_error(e);
        throw;
      } catch(...) {
        m_log->syslog("python", tu::log::error)<<"Unknown error while trying to create python reactor of type \""<<class_name<<"\".";
        throw;
      }
    }
    tu::SingletonUse<tu::LogManager> m_log;
    
  }; // python producer
  
  python_producer python_decl("PyReactor");
  
  
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
  
//  bool test_id(tt::graph::reactor_id const &id) {
//    return id;
//  }
//  tt::TeleoReactor const &reactor_id(tt::graph::reactor_id const &id) {
//    return *id;
//  }
//  std::string str_id(tt::graph::reactor_id const &id) {
//    if( id )
//      return id->getAgentName().str()+"."+id->getName().str();
//    return "";
//  }
  
  
  boost::shared_ptr<tt::Predicate> pred_factory(boost::property_tree::ptree::value_type &decl) {
    TREX::utils::SingletonUse<tt::Predicate::xml_factory> fact;
    return fact->produce(decl);
  }
  
  void python_add_reactor(tt::graph &g, boost::property_tree::ptree::value_type &cfg) {
    g.add_reactor(cfg);
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
  class_<tt::Predicate, boost::shared_ptr<tt::Predicate>,
         boost::noncopyable>("predicate", "trex timeline predicate", no_init)
  .def("object", &tt::Predicate::object, return_internal_reference<>())
  .def("predicate", &tt::Predicate::predicate, return_internal_reference<>())
  .def("has_attribute", &tt::Predicate::hasAttribute, arg("name"))
  .def("restrict", attr_1, arg("var"))
  .def("restrict", attr_2, (arg("name"), arg("domain")))
  .def("attribute", &tt::Predicate::getAttribute, return_internal_reference<>(), arg("name"))
  .def("xml", &xml_str<tt::Predicate>)
  .def("__str__", &str_impl<tt::Predicate>)
  .def("from_xml", &pred_factory).staticmethod("from_xml");
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
  
  class_<python_reactor, boost::shared_ptr<python_reactor>, boost::noncopyable>
  ("reactor", "abstract reactor interface", no_init)
  .def(init<tt::TeleoReactor::xml_arg_type &>())
  .add_property("name", make_function(&tt::TeleoReactor::getName, return_internal_reference<>()))
  .add_property("agent_name", make_function(&tt::TeleoReactor::getAgentName, return_internal_reference<>()))
  .add_property("graph", make_function(&tt::TeleoReactor::getGraph, return_internal_reference<>()))
  .add_property("initial_tick", &tt::TeleoReactor::getInitialTick)
  .add_property("current_tick", &tt::TeleoReactor::getCurrentTick)
  .add_property("final_tick", &tt::TeleoReactor::getFinalTick)
  .def("date_str", &tt::TeleoReactor::date_str)
  .add_property("latency", &tt::TeleoReactor::getLatency)
  .add_property("look_ahead", &tt::TeleoReactor::getLookAhead)
  .add_property("exec_latency", &tt::TeleoReactor::getExecLatency)
  .add_property("externals_count", &tt::TeleoReactor::count_externals)
  .add_property("internals_count", &tt::TeleoReactor::count_internals)
  .def("is_external", &python_reactor::ext_check)
  .def("use", &python_reactor::ext_use, (arg("tl"), arg("control")=true))
  .def("unuse", &python_reactor::ext_unuse)
  .def("request", &python_reactor::post_request)
  .def("recall", &python_reactor::cancel_request)
  .def("is_internal", &python_reactor::int_check)
  .def("declare", &python_reactor::int_decl, (arg("tl"), arg("control")=true))
  .def("undeclare", &python_reactor::int_undecl)
  .def("post", &python_reactor::post_obs, (arg("o"), arg("verbose")=false))
  // logging
  .def("info", &python_reactor::info)
  .def("warn", &python_reactor::warn)
  .def("error", &python_reactor::error)
  // transactions
  .def("notify", &tt::TeleoReactor::notify)
  .def("handle_request", &python_reactor::handleRequest)
  .def("handle_recall", &python_reactor::handleRecall)
  // execution
  .def("handle_init", &python_reactor::handleInit)
  .def("handle_new_tick", &python_reactor::handleTickStart)
  .def("synchronize", pure_virtual(&python_reactor::synchronize))
  .def("has_work", &python_reactor::hasWork)
  .def("resume", &python_reactor::resume)
  ;
  
  c_graph.def("add_reactor", &python_add_reactor);
}