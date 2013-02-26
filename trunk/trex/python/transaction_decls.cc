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
  
  
  struct reactor_wrap: tt::TeleoReactor, wrapper<tt::TeleoReactor> {
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
  
  class_<tt::graph, boost::noncopyable>("graph", "reactors transaction graph", no_init)
  .def("name", &tt::graph::getName, return_internal_reference<>())
  .def("empty", &tt::graph::empty)
  .add_property("reactors_count", &tt::graph::count_reactors)
  .add_property("relations_count", &tt::graph::count_relations)
  .add_property("current_tick", &tt::graph::getCurrentTick, "current tick date")
  .def("date_str", &tt::graph::date_str, "convert a tick into a date string")
  ;
  
  
}