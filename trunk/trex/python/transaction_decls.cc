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

namespace bp=boost::python;

using namespace TREX::python;
using namespace TREX::transaction;
using namespace TREX::utils;

namespace {
  
  /** @brief Convert domain to XML
   *
   * @tparam Obj The domain type
   *
   * @param[in] dom A domain
   *
   * Print the domain @p dom into its XML form
   *
   * @return The xml representation of @p dom
   *
   * @sa json_str
   */
  template<class Obj>
  std::string xml_str(Obj const &dom) {
    std::ostringstream oss;
    dom.toXml(oss);
    return oss.str();
  }

  /** @brief Convert domain to JSON
   *
   * @tparam Obj The domain type
   *
   * @param[in] dom A domain
   *
   * Print the domain @p dom into its JSON form
   *
   * @return The JSON representation of @p dom
   *
   * @sa xml_str
   */
  template<class Obj>
  std::string json_str(Obj const &dom) {
    std::ostringstream oss;
    dom.toJSON(oss);
    return oss.str();
  }
  /** @brief Print A domain value
   *
   * @tparam Obj A domain type
   *
   * @param[in] dom A domain
   *
   * Print the value of @p dom into a human readable string 
   *
   * @return A string representing the value of @p dom
   */
  template<class Obj>
  std::string str_impl(Obj const &dom) {
    std::ostringstream oss;
    oss<<dom;
    return oss.str();
  }

  /** @brief Predicate deserialization
   *
   * @param decl A ptree serialization for a predicate
   *
   * Parse the tree @pred ato create the corresponding predicate instance
   *
   * @return A predicate correspinding to the description in @p pred
   */
  SHARED_PTR<Predicate> pred_factory(boost::property_tree::ptree::value_type &decl) {
    TREX::utils::singleton::use<Predicate::xml_factory> fact;
    return fact->produce(decl);
  }
}

/** @brief Python declaration for trex.transaction module
 *
 * This function populates the python namespace trex.transaction with 
 * several classes and utilities corresponding to the TREX::transaction 
 * C++ namespace. It includes the following:
 * @li predicate A trex predicate
 * @li obs A trex observation (extends predicate)
 * @li goal A trex goal (extends predicate)
 * @li tick A trex tick date
 * @li graph A reactor transaction graph
 */
void export_transactions() {
  // Setup my submodule
  bp::object module(bp::handle<>(bp::borrowed(PyImport_AddModule("trex.transaction"))));
  bp::scope().attr("transaction") = module;
  bp::scope my_scope = module;
  // from now on everything is under trex.transaction
  
  void (Predicate::* attr_1)(Variable const &) = &Predicate::restrictAttribute;
  void (Predicate::* attr_2)(Symbol const &, DomainBase const &) = &Predicate::restrictAttribute;
  
  /*
   * class predicate:
   *  properties 
   *    - object : the name of the timeline
   *    - name : the preidcate name
   *  methods 
   *    - has_attribute(self, name) -> bool
   *    - attribute(self, name) -> domain
   *    - restrict(self, var)
   *    - restrict(self, name, domain)
   */
  bp::class_<Predicate, boost::shared_ptr<Predicate>, boost::noncopyable>
  ("predicate", "trex timeline predicate", bp::no_init)
  .add_property("object", bp::make_function(&Predicate::object, bp::return_internal_reference<>()))
  .add_property("name", bp::make_function(&Predicate::predicate, bp::return_internal_reference<>()))
  .def("has_attribute", &Predicate::hasAttribute, (bp::arg("name")))
  .def("attribute", &Predicate::getAttribute, bp::return_internal_reference<>(), (bp::arg("name")))
  .def("restrict", attr_1, (bp::arg("var")))
  .def("restrict", attr_2, (bp::arg("name"), bp::arg("domain")))
  .def("xml", &xml_str<Predicate>)
  .def("json", &json_str<Predicate>)
  .def("from_xml", &pred_factory).staticmethod("from_xml");
  ;
  
  
  /*
   * class obs(predicate):
   *  methods
   *    - __init__(self, predicate)
   *    - __init__(self, symbol, symbol)
   */
  bp::class_<Observation, observation_id, bp::bases<Predicate> >
  ("obs", "trex observation", bp::init<Predicate const &>(bp::args("pred"), "Convert pred into an observation"))
  .def(bp::init<Symbol, Symbol>(bp::args("timeline", "pred"),
                                "Create new observation pred on timeline"))
  ;
  
  /*
   * class goal(predicate):
   *  methods
   *    - __init__(self, symbol, symbol)
   */
  bp::class_<Goal, goal_id, bp::bases<Predicate> >
  ("goal", "trex goal", bp::init<Symbol, Symbol>(bp::args("timeline", "pred"),
                                                 "Create new goal pred on timeline"));

  
  bp::class_<TICK, boost::noncopyable>("tick", "trex tick date", bp::init<>())
  .def(bp::init<long>())
  .def("__str__", &str_impl<TICK>)
  .def(bp::self == bp::self)
  .def(bp::self != bp::self)
  .def(bp::self < bp::self)
  .def(bp::self <= bp::self)
  .def(bp::self > bp::self)
  .def(bp::self >= bp::self)
  ;
  
  bp::implicitly_convertible<long, TICK>();
  
  
  bp::class_<graph, boost::noncopyable> c_graph("graph", "reactors transaction graph", bp::no_init);
  
  
  
  c_graph.def("name", &graph::getName, bp::return_internal_reference<>())
  .add_property("empty", &graph::empty)
  .add_property("reactors_count", &graph::count_reactors)
  .add_property("relations_count", &graph::count_relations)
  .add_property("current_tick", &graph::getCurrentTick, "current tick date")
  .def("date_str", &graph::date_str, "convert a tick into a date string")
  ;

}

/*
using namespace boost::python;

namespace tu=TREX::utils;
namespace tt=TREX::transaction;

namespace {
 
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
    TREX::utils::singleton::use<tt::Predicate::xml_factory> fact;
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
*/