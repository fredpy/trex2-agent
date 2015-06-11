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
  template<class Obj>
  std::string xml_str(Obj const &dom) {
    std::ostringstream oss;
    dom.toXml(oss);
    return oss.str();
  }

  template<class Obj>
  std::string json_str(Obj const &dom) {
    std::ostringstream oss;
    dom.toJSON(oss);
    return oss.str();
  }
  template<class Obj>
  std::string str_impl(Obj const &dom) {
    std::ostringstream oss;
    oss<<dom;
    return oss.str();
  }

  SHARED_PTR<Predicate> pred_factory(boost::property_tree::ptree::value_type &decl) {
    TREX::utils::SingletonUse<Predicate::xml_factory> fact;
    return fact->produce(decl);
  }
  
  void python_add_reactor(graph &g,
                          boost::property_tree::ptree::value_type &cfg) {
    g.add_reactor(cfg);
  }

}

void export_transactions() {
  // Setup my submodule
  bp::object module(bp::handle<>(bp::borrowed(PyImport_AddModule("trex.transaction"))));
  bp::scope().attr("transaction") = module;
  bp::scope my_scope = module;
  // from now on everything is under trex.transaction
  
  bp::docstring_options doc_options(true, true, false);
  
  module.attr("__doc__") = "Trex message transaction classes.\n"
  "The classes that are used to connect and handle Trex reactors\n"
  "message exchange, including the reactor abstract class itself.\n"
  "All the components here reflect the C++ library TREXtransaction"
  ;
  
  
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
  ("predicate",
   "Trex timeline predicate.\n\n"
   "A predicate is the basic message used by trex. It can\n"
   "be seen as a flat structure with multiple variables as\n"
   "attributes. A predicate is defined by its type (name here)\n"
   " and its associated object.", bp::no_init)
  .add_property("object", bp::make_function(&Predicate::object, bp::
                                            return_internal_reference<>()),
                "The object this predicate refers to. In Trex terminology\n"
                "this object is also referred to as the timeline.")
  .add_property("name", bp::make_function(&Predicate::predicate, bp::
                                          return_internal_reference<>()),
                "The name (or type) of this predicate")
  .def("has_attribute", &Predicate::hasAttribute, (bp::args("self", "name")),
       "Check if this predicate has the attribute name.\n"
       "NOTE: In Trex, having an attribute only means that this attribute\n"
       "      has been constrained somehow. The lack of an attribute is,\n"
       "      by convention, similar to have this attribute with no\n"
       "      constraint on its domain.")
  .def("attribute", &Predicate::getAttribute,
       bp::return_internal_reference<>(), (bp::args("self", "name")),
       "Access the attribute name.\n"
       "precondition: self.has_attribute(name)")
  .def("restrict", attr_1, (bp::args("self", "var")),
       "Restrict the attribute named var.name to var.domain.\n"
       "if var.name did not exist then just set it to var.domain\n"
       "otherwise compute the intersection with var.domain\n"
       "error: if this operation result on var.name attribute's\n"
       "       domain to become empty")
  .def("restrict", attr_2, bp::args("self", "name","domain"),
       "Restrict the attribute named name to domain.\n"
       "if var.name did not exist then just set it to domain\n"
       "otherwise compute the intersection with domain\n"
       "error: if this operation result on name attribute's\n"
       "       domain to become empty")
  .def("xml", &xml_str<Predicate>, bp::arg("self"),
       "Serialize this predicate as an xml formatted string")
  .def("json", &json_str<Predicate>, bp::arg("self"),
       "Serialize this predicate as a json formatted string")
  .def("from_xml", &pred_factory, bp::arg("xml"),
       "Create a new instance from an xml definition")
  .staticmethod("from_xml")
  ;
  
  
  /*
   * class obs(predicate):
   *  methods
   *    - __init__(self, predicate)
   *    - __init__(self, symbol, symbol)
   */
  bp::class_<Observation, observation_id, bp::bases<Predicate> >
  ("obs", "trex observation.\n\n"
   "A Trex observation is a predicate that applies to the current tick.\n"
   "It is the state value of the timeline self.object",
   bp::init<Predicate const &>(bp::args("self", "pred"),
                               "Convert the predicate pred into an observation"))
  .def(bp::init<Symbol, Symbol>(bp::args("self", "timeline", "pred"),
                                "Create new observation pred on timeline"))
  ;
  
  /*
   * class goal(predicate):
   *  methods
   *    - __init__(self, symbol, symbol)
   */
  bp::class_<Goal, goal_id, bp::bases<Predicate> >
  ("goal", "trex goal.\n\n"
   "A trex goal is a predicate with the special attributes\n"
   "start, duration and end that indicate its temporal scope.\n"
   "It repsresent a desired future state for the timeline\n"
   "self.object and is used for sending requests to the reactor\n"
   " that provide this timline",
   bp::init<Symbol, Symbol>(bp::args("self", "timeline", "pred"),
                            "Create new goal pred on timeline"));

  
  bp::class_<TICK, boost::noncopyable>
  ("tick", "Trex tick date."
   "The tick is the basic reprsentaion of time for trex\n"
   "It is an integer that is updated by the clock as time advance.",
   bp::init<>(bp::arg("self"),
              "Create a new instance with the value 0"))
  .def(bp::init<long>(bp::args("self", "val"),
                      "Create a new instance with the value val"))
  .def("__str__", &str_impl<TICK>, bp::arg("self"),
       "serialize this instance into a human readable string")
  .def(bp::self == bp::self)
  .def(bp::self != bp::self)
  .def(bp::self < bp::self)
  .def(bp::self <= bp::self)
  .def(bp::self > bp::self)
  .def(bp::self >= bp::self)
  ;
  
  bp::implicitly_convertible<long, TICK>();
  
  
  bp::class_<graph, boost::noncopyable>
  c_graph("graph",
          "Reactors transaction graph.\n\n"
          "The basic structure that connect the reatcors together\n"
          "into a graph and handle message exchanges between these\n"
          "reactors based on this graph.\n"
          "A reactor graph is defined by what timelines each reactor\n"
          "use (declare as external) and provide (declare as internal).\n"
          "For each timeline there can be only one reactor providing it\n"
          "but many can use it as long as it does not generate cyclic\n"
          "dependency between reactors. This structure is what manage\n"
          "these connections internally.",
          bp::no_init);
  
  c_graph.add_property("name",
                       bp::make_function(&graph::getName,
                                         bp::return_internal_reference<>()),
                       "The name of the graph")
  .add_property("empty", &graph::empty,
                "Check if empty (i.e no reactor)")
  .add_property("reactors_count", &graph::count_reactors,
                "Number of reactors in this graph")
  .add_property("relations_count", &graph::count_relations,
                "Number of connections between reactors in this graph")
  .add_property("current_tick", &graph::getCurrentTick, "current tick date")
  .def("date_str", &graph::date_str, bp::args("self", "tick"),
       "convert a tick into a date string")
  ;
  
  bp::class_<py_wrapper>
  ("reactor_anchor", "Internal anchor for a python reactor", bp::no_init);
  
  bp::class_<reactor_wrap, boost::noncopyable>
  ("reactor", "Python api for reactor.\n"
   "This class allow user to define their own reactor\n"
   "class in python. The instance can then be created in\n"
   "a Trex agent (or graph) through the xml tag of the form:\n"
   "<PyReactor name=\"name\" python_class=\"<class_type>\"\n"
   "           latency=\"latency\" lookahead=\"lookahead\"/>\n"
   "\n"
   "Where <class_type> is the class that derives from this\n"
   "abstract reactor class.",
   bp::init<py_wrapper const &>(bp::args("self", "anchor"),
                                "Create a new instance.\n"
                                "NOTE: this constructor needs to be redefined and called on\n"
                                "       each of its derived classes!"))
  .add_property("name",
                bp::make_function(&reactor_proxy::name,
                                  bp::return_internal_reference<>()),
                "The reactor name")
  .add_property("latency", &reactor_proxy::latency,
                "The reactor latency")
  .add_property("lookahead", &reactor_proxy::lookahead,
                "The reactor lookahead")
  .add_property("exec_latency", &reactor_proxy::exec_latency,
                "The reactor execution latency (i.e. its latency\n"
                "increased by the latency of the timelines this\n"
                "reactor uses.")
  .add_property("initial_tick", &reactor_proxy::initial,
                "initial tick (usually 0)")
  .add_property("final_tick", &reactor_proxy::final,
                "final tick after which this reactor lifetime will end")
  .add_property("tick", &reactor_proxy::current,
                "current tick date")
  .def("date_str", &reactor_proxy::date_str, bp::args("self", "tick"),
       "Convert a tick into a human readbale date")
  .def("use", &reactor_proxy::use_tl, (bp::arg("self"),
                                       bp::arg("tl"),
                                       bp::arg("control")=true),
       "Request to use timeline tl as external")
  .def("provide", &reactor_proxy::provide_tl, (bp::arg("self"),
                                               bp::arg("tl"),
                                               bp::arg("control")=true),
       "Request to provide timeline tl as internal")
  .def("is_internal", &reactor_proxy::is_internal, bp::args("self", "tl"),
       "Check if tl is internal (provided) by this reactor")
  .def("is_external", &reactor_proxy::is_external, bp::args("self", "tl"),
       "Check if tl is external (used) by this reactor")
  .def("post", &reactor_proxy::post, (bp::arg("self"),
                                      bp::arg("o"),
                                      bp::arg("verbose")=false),
       "Post the observation o for being published after the end of\n"
       " the next synchronization.\n"
       "precondition: o.object is internal to this reactor\n"
       "NOTE: in case of multiple posts on the same timeline\n"
       "      only the last post will be published.")
  .def("notify", &reactor_proxy::notify, &reactor_wrap::notify_default,
       bp::args("self", "o"),
       "This method is called whenever a new observation has\n"
       "been published on a external timeline (used) of this\n"
       "reactor. The observation is given by o")
  .def("synchronize", bp::pure_virtual(&reactor_wrap::synchronize),
       bp::arg("self"),
       "Method called by the agent at the beginning of any new\n"
       "tick. This method is typically used to produce new \n"
       "observations for this reactor internal timelines.\n"
       "If this method return False this will result on the\n"
       "reactor being killed by the agent.\n"
       "NOTE: Implementing this method is required for any new\n"
       "      derived classes."
       )
  ;
  
  
  
//  bp::class_<python_reactor, SHARED_PTR<python_reactor>, boost::noncopyable>
//  ("reactor", "abstract reactor interface", bp::no_init)
//  .def(bp::init<TeleoReactor::xml_arg_type &>())
//  .add_property("name", make_function(&python_reactor::getName, bp::return_internal_reference<>()))
////  .add_property("agent_name", make_function(&TeleoReactor::getAgentName, bp::return_internal_reference<>()))
////  .add_property("graph", make_function(&TeleoReactor::getGraph, bp::return_internal_reference<>()))
////  .add_property("initial_tick", &TeleoReactor::getInitialTick)
////  .add_property("current_tick", &TeleoReactor::getCurrentTick)
////  .add_property("final_tick", &TeleoReactor::getFinalTick)
////  .def("date_str", &TeleoReactor::date_str)
////  .add_property("latency", &TeleoReactor::getLatency)
////  .add_property("look_ahead", &TeleoReactor::getLookAhead)
////  .add_property("exec_latency", &TeleoReactor::getExecLatency)
////  .add_property("externals_count", &TeleoReactor::count_externals)
////  .add_property("internals_count", &TeleoReactor::count_internals)
////  .def("is_external", &python_reactor::ext_check)
////  .def("use", &python_reactor::ext_use, (bp::arg("tl"),
////                                         bp::arg("control")=true))
////  .def("unuse", &python_reactor::ext_unuse)
////  .def("request", &python_reactor::post_request)
////  .def("recall", &python_reactor::cancel_request)
////  .def("is_internal", &python_reactor::int_check)
//  .def("declare", &python_reactor::int_decl, (bp::arg("tl"),
//                                              bp::arg("control")=true))
////  .def("undeclare", &python_reactor::int_undecl)
////  .def("post", &python_reactor::post_obs, (bp::arg("o"),
////                                           bp::arg("verbose")=false))
////  // logging
////  .def("info", &python_reactor::info)
////  .def("warn", &python_reactor::warn)
////  .def("error", &python_reactor::error)
////  // transactions
////  .def("notify", &TeleoReactor::notify)
////  .def("handle_request", &python_reactor::handleRequest)
////  .def("handle_recall", &python_reactor::handleRecall)
////  // execution
////  .def("handle_init", &python_reactor::handleInit)
////  .def("handle_new_tick", &python_reactor::handleTickStart)
//  .def("synchronize", pure_virtual(&python_reactor::synchronize))
////  .def("has_work", &python_reactor::hasWork)
////  .def("resume", &python_reactor::resume)
//  ;
  
  c_graph.def("add_reactor", &python_add_reactor, bp::args("self", "xml"));
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
*/