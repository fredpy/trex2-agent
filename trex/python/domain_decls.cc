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
#include <trex/domain/boolean_domain.hh>
#include <trex/domain/int_domain.hh>
#include <trex/domain/float_domain.hh>
#include <trex/domain/enum_domain.hh>
#include <trex/domain/string_domain.hh>
#include <trex/domain/var.hh>

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

using namespace boost::python;

namespace tu=TREX::utils;
namespace tt=TREX::transaction;

namespace {
  
  struct domain_wrapper:tt::abstract_domain, wrapper<tt::abstract_domain> {
    domain_wrapper(tu::symbol const &name)
    :TREX::transaction::abstract_domain(name) {}
    ~domain_wrapper() {}
    
    tt::abstract_domain *copy() const {
      return this->get_override("copy")();
    }

    bool is_interval() const {
      return this->get_override("is_interval")();
    }
    bool is_enumerated() const {
      return this->get_override("is_enumerated")();
    }
    
    bool is_full() const {
      return this->get_override("is_full")();
    }
    bool is_singleton() const {
      return this->get_override("is_singleton")();
    }

    bool intersect(tt::abstract_domain const &other) const {
      return this->get_override("intersect")(other);
    }

    bool equals(tt::abstract_domain const &other) const {
      return this->get_override("__eq__")(other);
    }
    
    tt::abstract_domain &restrict_with(tt::abstract_domain const &other) {
      return this->get_override("restrict")(other);
    }
  
    boost::property_tree::ptree build_tree() const {
      return this->get_override("build_tree")();
    }
    
    std::string str() const {
      return this->get_override("__str__")();
    }
    
    std::ostream &print_domain(std::ostream &out) const {
      return out<<str();
    }

    boost::any singleton() const {
      return boost::any();
    }
    
    std::string string_singleton() const {
      if( is_singleton() )
        return str();
      return std::string();
    }
  };
  
  
  struct interval_wrap:tt::basic_interval, wrapper<tt::basic_interval> {
    interval_wrap(tu::symbol const &type):tt::basic_interval(type) {}
    
    tt::abstract_domain *copy() const {
      return this->get_override("copy")();
    }
    bool is_singleton() const {
      return this->get_override("is_singleton")();
    }
    bool has_lower()  const {
      return this->get_override("has_lower")();
    }
    bool has_upper()  const {
      return this->get_override("has_upper")();
    }
    bool equals(tt::abstract_domain const &other) const {
      return this->get_override("__eq__")(other);
    }
    
    bool intersect(tt::abstract_domain const &other) const {
      return this->get_override("intersect")(other);
    }

    boost::any get_lower() const {
      return boost::any();
    }
    boost::any get_upper() const {
      return boost::any();
    }
    tt::abstract_domain &restrict_with(tt::abstract_domain const &other) {
      return this->get_override("restrict")(other);
    }
    void parse_lower(std::string const &val) {
      this->get_override("parse_low")(val);
    }
    void parse_upper(std::string const &val) {
      this->get_override("parse_up")(val);
    }
    void parse_singleton(std::string const &val) {
      this->get_override("parse_singleton")(val);
    }

    std::string str_low() const {
      return this->get_override("str_low")();
    }
    std::string str_up() const {
      return this->get_override("str_up")();
    }
    
    std::ostream &print_lower(std::ostream &out) const {
      return out<<str_low();
    }
    std::ostream &print_upper(std::ostream &out) const {
      return out<<str_up();
    }

    
  };
  
  struct enum_wrap: tt::basic_enumerated, wrapper<tt::basic_enumerated> {
    enum_wrap(tu::symbol const &type):tt::basic_enumerated(type) {}
    
    tt::abstract_domain *copy() const {
      return this->get_override("copy")();
    }
    bool intersect(tt::abstract_domain const &other) const {
      return this->get_override("intersect")(other);
    }
    
    bool equals(tt::abstract_domain const &other) const {
      return this->get_override("__eq__")(other);
    }

    size_t size() const {
      return this->get_override("__len__")();
    }
    boost::any element(size_t) const {
      return boost::any();
    }
    tt::abstract_domain &restrict_with(tt::abstract_domain const &other) {
      return this->get_override("restrict")(other);
    }
    
    virtual void add_string_value(std::string const &val) {
      this->get_override("add")(val);
    }
    
    std::string get_text(size_t i) const {
      return this->get_override("get")(i);
    }
    
    std::ostream &print_value(std::ostream &out, size_t i) const {
      return out<<get_text(i);
    }
  
  };
  
  
  
  template<class Obj>
  std::string xml_str(Obj const &dom) {
    std::ostringstream oss;
    dom.to_xml(oss);
    return oss.str();
  }
  
  template<class Obj>
  std::string json_str(Obj const &dom) {
    std::ostringstream oss;
    dom.to_json(oss);
    return oss.str();
  }

  template<class Obj>
  std::string str_impl(Obj const &dom) {
    std::ostringstream oss;
    oss<<dom;
    return oss.str();
  }
  
  template<class Enum, class Base>
  Enum collection_init(object o) {
    stl_input_iterator<Base> start(o), finish;
    return Enum(start, finish);
  }

}

void export_domain() {
  // Setup my submodule
  object module(handle<>(borrowed(PyImport_AddModule("trex.domains"))));
  scope().attr("domains") = module;
  scope my_scope = module;
  // from now on eveerything is under trex.domain
  
  
  // class trex.domains.domain
  //  abstract domain interface
  //    - type()            give a string representation of the type
  //    - is_interval()     indicates if the domain is an interval
  //    - is_enumerated()   indicates if the domain is enumerated
  //    - is_full()         indicates if the domain is full
  //    - is_singleton()    indicates if the domain is a singleton
  //    - intersect(domain) checks if the intersection with the arg is not empty
  //    - __eq__(domain)    check if identical to the argument
  //    - restrict(domain)  instersects with the arg
  //    - xml()             gives xml repredsentation
  //    - __str__()         gives the string representation
  class_<domain_wrapper, boost::noncopyable>("domain", "Abstract trex domain",
                                             init<tu::symbol>())
  .def("type", &tt::abstract_domain::type_name, return_internal_reference<>())
  .def("is_interval", pure_virtual(&tt::abstract_domain::is_interval))
  .def("is_enumerated", pure_virtual(&tt::abstract_domain::is_enumerated))
  .def("is_full", pure_virtual(&tt::abstract_domain::is_full))
  .def("is_singleton", pure_virtual(&tt::abstract_domain::is_singleton))
  .def("intersect", pure_virtual(&tt::abstract_domain::intersect))
  .def("__eq__", pure_virtual(&tt::abstract_domain::equals))
  .def("restrict", pure_virtual(&tt::abstract_domain::restrict_with),
       return_internal_reference<>())
  .def("as_tree", &tt::abstract_domain::as_tree)
  .def("build_tree", pure_virtual(&tt::abstract_domain::build_tree))
  .def("xml", &xml_str<tt::abstract_domain>)
  .def("json", &json_str<tt::abstract_domain>)
  .def("__str__", pure_virtual(&str_impl<tt::abstract_domain>))
  ;
  
  // class trex.domains.interval: trex.domains.domain
  //  abstract interval interface
  //    - has_lower() check if interval has a non infinite lower bound
  //    - has_upper() check if interval has a non infinite upper bound
  class_<interval_wrap, bases<tt::abstract_domain>,
         boost::noncopyable>("interval", "Abstract trex interval",
                             init<tu::symbol>())
  .def("has_lower", pure_virtual(&tt::basic_interval::has_lower))
  .def("has_upper", pure_virtual(&tt::basic_interval::has_upper))
  ;
  
  // class trex.domains.bool: trex.domains.interval
  //  boolean domain
  //   - __init__()     create a full boolean domain
  //   - __init__(bool) create a domain with the single value arg
  class_<tt::boolean_domain, bases<tt::basic_interval> >("bool", "Boolean domain",
                                                    init<>())
  .def(init<bool>())
  ;
  
  // class trex.domains.int: trex.domains.interval
  //  integer domain
  //   - __init__()     create a full int domain
  //   - __init__(long) create a domain with the single value arg
  //   - __init__(long, long) create a domain with interval [arg1, arg2]
  class_<tt::int_domain, bases<tt::basic_interval> >("int", "Integer domain", init<>())
  .def(init<long>())
  .def(init<long,long>())
  ;

  // class trex.domains.float: trex.domains.interval
  //  float domain
  //   - __init__()     create a full int domain
  //   - __init__(double) create a domain with the single value arg
  //   - __init__(double, double) create a domain with interval [arg1, arg2]
  class_<tt::float_domain, bases<tt::basic_interval> >("float", "Float domain", init<>())
  .def(init<double>())
  .def(init<double,double>())
  ;
  
  // class trex.domains.enumerated: trex.domains.domain
  // abstract enumerated domain interface
  //   - __len__()    number of elements
  class_<enum_wrap, bases<tt::abstract_domain>,
         boost::noncopyable>("enumerated", "Abstract trex Enumerated domain",
                             init<tu::symbol>())
  .def("__len__", pure_virtual(&tt::basic_enumerated::size))
  // TODO need to implement __iter__
  ;
  
  
  // class trex.domains.string: trex.domains.enumerated
  //  string domain
  //    - __init__()           create the full domain
  //    - __init__(string)     create a domain with the single value arg
  //    - __init__(collection) create a domain with the elements given in colllection
  class_<tt::string_domain, bases<tt::basic_enumerated> >("string", "string domain", init<>())
  .def(init<std::string>())
  .def("__init__", &collection_init<tt::string_domain, std::string>)
  ;

  
  // class trex.domains.enum: trex.domains.enumerated
  //  enum domain
  //    - __init__()                  create the full domain
  //    - __init__(trex.utils.symbol) create a domain with the single value arg
  //    - __init__(collection)        create a domain with the elements given in collection
  class_<tt::enum_domain, bases<tt::basic_enumerated> >("enum", "enum domain", init<>())
  .def(init<tu::symbol>())
  .def("__init__", &collection_init<tt::enum_domain, tu::symbol>)
  ;
  
  tt::var &(tt::var::* restrict_domain)(tt::abstract_domain const &) = &tt::var::restrict_with;
  tt::var &(tt::var::* restrict_var)(tt::var const &) = &tt::var::restrict_with;
  
  // class trex.domains.var
  //   A trex variable
  //    - __init__(trex.utils.symbol, trex.domains.domain) create a variable with symbol as name and domain as base domain
  //    - name() get variable name
  //    - domain() get variable domain
  //    - restrict(domain) restrict the varaiable by domain
  //    - restrict(var) restrict variable with var.domain()
  //    - xml()     xml representation
  //    - __str__() string representation
  class_<tt::var>("var", "trex variable",
                  init<tu::symbol, tt::abstract_domain const &>())
  .def("name", &tt::var::name)
  .def("domain", &tt::var::domain, return_internal_reference<>())
  .def("restrict", restrict_domain, return_internal_reference<>())
  .def("restrict", restrict_var, return_internal_reference<>())
  .def("xml", &xml_str<tt::var>)
  .def("json", &json_str<tt::var>)
  .def("__str__", &str_impl<tt::var>)
  ;

}




