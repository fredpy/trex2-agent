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
#include <trex/domain/BooleanDomain.hh>
#include <trex/domain/IntegerDomain.hh>
#include <trex/domain/FloatDomain.hh>
#include <trex/domain/EnumDomain.hh>
#include <trex/domain/StringDomain.hh>
#include <trex/domain/Variable.hh>

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

using namespace boost::python;

namespace tu=TREX::utils;
namespace tt=TREX::transaction;

namespace {
  
  struct domain_wrapper:tt::DomainBase, wrapper<tt::DomainBase> {
    domain_wrapper(tu::Symbol const &name)
    :TREX::transaction::DomainBase(name) {}
    ~domain_wrapper() {}
    
    tt::DomainBase *copy() const {
      return this->get_override("copy")();
    }

    bool isInterval() const {
      return this->get_override("is_interval")();
    }
    bool isEnumerated() const {
      return this->get_override("is_enumerated")();
    }
    
    bool isFull() const {
      return this->get_override("is_full")();
    }
    bool isSingleton() const {
      return this->get_override("is_singleton")();
    }

    bool intersect(tt::DomainBase const &other) const {
      return this->get_override("intersect")(other);
    }

    bool equals(tt::DomainBase const &other) const {
      return this->get_override("__eq__")(other);
    }
    
    tt::DomainBase &restrictWith(tt::DomainBase const &other) {
      return this->get_override("restrict")(other);
    }
  
    std::string xml() const {
      return this->get_override("xml")();
    }
    
    std::ostream &toXml(std::ostream &out, size_t) const {
      return out<<xml();
    }
    
    std::string json() const {
      return this->get_override("json")();
    }
    std::ostream &toJSON(std::ostream &out, size_t) const {
      return out<<json();
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
    
    std::string stringSingleton() const {
      if( isSingleton() )
        return str();
      return std::string();
    }
  };
  
  
  struct interval_wrap:tt::BasicInterval, wrapper<tt::BasicInterval> {
    interval_wrap(tu::Symbol const &type):tt::BasicInterval(type) {}
    
    tt::DomainBase *copy() const {
      return this->get_override("copy")();
    }
    bool isSingleton() const {
      return this->get_override("is_singleton")();
    }
    bool hasLower()  const {
      return this->get_override("has_lower")();
    }
    bool hasUpper()  const {
      return this->get_override("has_upper")();
    }
    bool equals(tt::DomainBase const &other) const {
      return this->get_override("__eq__")(other);
    }
    
    bool intersect(tt::DomainBase const &other) const {
      return this->get_override("intersect")(other);
    }

    boost::any getLower() const {
      return boost::any();
    }
    boost::any getUpper() const {
      return boost::any();
    }
    tt::DomainBase &restrictWith(tt::DomainBase const &other) {
      return this->get_override("restrict")(other);
    }
    void parseLower(std::string const &val) {
      this->get_override("parse_low")(val);
    }
    void parseUpper(std::string const &val) {
      this->get_override("parse_up")(val);
    }
    void parseSingleton(std::string const &val) {
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
  
  struct enum_wrap: tt::BasicEnumerated, wrapper<tt::BasicEnumerated> {
    enum_wrap(tu::Symbol const &type):tt::BasicEnumerated(type) {}
    
    tt::DomainBase *copy() const {
      return this->get_override("copy")();
    }
    bool intersect(tt::DomainBase const &other) const {
      return this->get_override("intersect")(other);
    }
    
    bool equals(tt::DomainBase const &other) const {
      return this->get_override("__eq__")(other);
    }

    size_t getSize() const {
      return this->get_override("__len__")();
    }
    boost::any getElement(size_t) const {
      return boost::any();
    }
    tt::DomainBase &restrictWith(tt::DomainBase const &other) {
      return this->get_override("restrict")(other);
    }
    
    virtual void addTextValue(std::string const &val) {
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
                                             init<tu::Symbol>())
  .def("type", &tt::DomainBase::getTypeName, return_internal_reference<>())
  .def("is_interval", pure_virtual(&tt::DomainBase::isInterval))
  .def("is_enumerated", pure_virtual(&tt::DomainBase::isEnumerated))
  .def("is_full", pure_virtual(&tt::DomainBase::isFull))
  .def("is_singleton", pure_virtual(&tt::DomainBase::isSingleton))
  .def("intersect", pure_virtual(&tt::DomainBase::intersect))
  .def("__eq__", pure_virtual(&tt::DomainBase::equals))
  .def("restrict", pure_virtual(&tt::DomainBase::restrictWith),
       return_internal_reference<>())
  .def("xml", pure_virtual(&xml_str<tt::DomainBase>))
  .def("json", pure_virtual(&json_str<tt::DomainBase>))
  .def("__str__", pure_virtual(&str_impl<tt::DomainBase>))
  ;
  
  // class trex.domains.interval: trex.domains.domain
  //  abstract interval interface
  //    - has_lower() check if interval has a non infinite lower bound
  //    - has_upper() check if interval has a non infinite upper bound
  class_<interval_wrap, bases<tt::DomainBase>,
         boost::noncopyable>("interval", "Abstract trex interval",
                             init<tu::Symbol>())
  .def("has_lower", pure_virtual(&tt::BasicInterval::hasLower))
  .def("has_upper", pure_virtual(&tt::BasicInterval::hasUpper))
  ;
  
  // class trex.domains.bool: trex.domains.interval
  //  boolean domain
  //   - __init__()     create a full boolean domain
  //   - __init__(bool) create a domain with the single value arg
  class_<tt::BooleanDomain, bases<tt::BasicInterval> >("bool", "Boolean domain",
                                                    init<>())
  .def(init<bool>())
  ;
  
  // class trex.domains.int: trex.domains.interval
  //  integer domain
  //   - __init__()     create a full int domain
  //   - __init__(long) create a domain with the single value arg
  //   - __init__(long, long) create a domain with interval [arg1, arg2]
  class_<tt::IntegerDomain, bases<tt::BasicInterval> >("int", "Integer domain", init<>())
  .def(init<long>())
  .def(init<long,long>())
  ;

  // class trex.domains.float: trex.domains.interval
  //  float domain
  //   - __init__()     create a full int domain
  //   - __init__(double) create a domain with the single value arg
  //   - __init__(double, double) create a domain with interval [arg1, arg2]
  class_<tt::FloatDomain, bases<tt::BasicInterval> >("float", "Float domain", init<>())
  .def(init<double>())
  .def(init<double,double>())
  ;
  
  // class trex.domains.enumerated: trex.domains.domain
  // abstract enumerated domain interface
  //   - __len__()    number of elements
  class_<enum_wrap, bases<tt::DomainBase>,
         boost::noncopyable>("enumerated", "Abstract trex Enumerated domain",
                             init<tu::Symbol>())
  .def("__len__", pure_virtual(&tt::BasicEnumerated::getSize))
  // TODO need to implement __iter__
  ;
  
  
  // class trex.domains.string: trex.domains.enumerated
  //  string domain
  //    - __init__()           create the full domain
  //    - __init__(string)     create a domain with the single value arg
  //    - __init__(collection) create a domain with the elements given in colllection
  class_<tt::StringDomain, bases<tt::BasicEnumerated> >("string", "string domain", init<>())
  .def(init<std::string>())
  .def("__init__", &collection_init<tt::StringDomain, std::string>)
  ;

  
  // class trex.domains.enum: trex.domains.enumerated
  //  enum domain
  //    - __init__()                  create the full domain
  //    - __init__(trex.utils.symbol) create a domain with the single value arg
  //    - __init__(collection)        create a domain with the elements given in collection
  class_<tt::EnumDomain, bases<tt::BasicEnumerated> >("enum", "enum domain", init<>())
  .def(init<tu::Symbol>())
  .def("__init__", &collection_init<tt::EnumDomain, tu::Symbol>)
  ;
  
  tt::Variable &(tt::Variable::* restrict_domain)(tt::DomainBase const &) = &tt::Variable::restrict;
  tt::Variable &(tt::Variable::* restrict_var)(tt::Variable const &) = &tt::Variable::restrict;
  
  // class trex.domains.var
  //   A trex variable
  //    - __init__(trex.utils.symbol, trex.domains.domain) create a variable with symbol as name and domain as base domain
  //    - name() get variable name
  //    - domain() get variable domain
  //    - restrict(domain) restrict the varaiable by domain
  //    - restrict(var) restrict variable with var.domain()
  //    - xml()     xml representation
  //    - __str__() string representation
  class_<tt::Variable>("var", "trex variable", init<tu::Symbol, tt::DomainBase const &>())
  .def("name", &tt::Variable::name, return_internal_reference<>())
  .def("domain", &tt::Variable::domain, return_internal_reference<>())
  .def("restrict", restrict_domain, return_internal_reference<>())
  .def("restrict", restrict_var, return_internal_reference<>())
  .def("xml", &xml_str<tt::Variable>)
  .def("json", &json_str<tt::Variable>)
  .def("__str__", &str_impl<tt::Variable>)
  ;

}




