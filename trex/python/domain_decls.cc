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
#include <trex/domain/BooleanDomain.hh>
#include <trex/domain/IntegerDomain.hh>
#include <trex/domain/FloatDomain.hh>
#include <trex/domain/EnumDomain.hh>
#include <trex/domain/StringDomain.hh>
#include <trex/domain/Variable.hh>

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

#include "exception_helper.hh"

using namespace boost::python;

namespace tu=TREX::utils;
namespace tt=TREX::transaction;

namespace {
  
  tu::SingletonUse<TREX::python::exception_table>  s_py_err;

  
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
  
  std::string get_elem_str(tt::BasicEnumerated const &a,
                           size_t idx) {
    if( idx>=a.getSize() ) {
      std::ostringstream oss;
      oss<<"Index "<<idx<<" is out of bounds";
      throw tt::DomainAccess(a, oss.str());
    }
    return a.getStringValue(idx);
  }
  
  tt::Variable var_from_xml(boost::property_tree::ptree::value_type &xml) {
    return tt::Variable(xml);
  }
}

void export_domain() {
  // Setup my submodule
  object module(handle<>(borrowed(PyImport_AddModule("trex.domains"))));
  scope().attr("domains") = module;
  scope my_scope = module;
  // from now on eveerything is under trex.domain
  
  docstring_options doc_options(true, true, false);
  
  module.attr("__doc__") = "Trex domains.\n"
  "The classes that are used by trex to represent values. Taking its \n"
  "root from constraint programming many variables in trex -- specifically\n"
  "the ones in predicates, observations and goals -- are represented as\n"
  "domains. A domain is a set of possible values either represented as an\n"
  "interval or set.\n"
  "All these objects are related to components from TREXdomain C++ library."
  ;
  
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
  class_<domain_wrapper, boost::noncopyable>
  ("domain", "Abstract trex domain",
   init<tu::Symbol>(args("self")))
  .add_property("type", make_function(&tt::DomainBase::getTypeName,
                                      return_internal_reference<>()),
                "the type represented by this domain")
  .def("is_interval", pure_virtual(&tt::DomainBase::isInterval),
       arg("self"),
       "Check if the domain is implemented as an interval")
  .def("is_enumerated", pure_virtual(&tt::DomainBase::isEnumerated),
       arg("self"),
       "Check if the domain is implemented as an anumeration of values")
  .def("is_full", pure_virtual(&tt::DomainBase::isFull), arg("self"),
       "Check if the domain is full (i.e. contains all possible values\n"
       "for this type).")
  .def("is_singleton", pure_virtual(&tt::DomainBase::isSingleton), arg("self"),
       "Check if the domain is a singleton (i.e. contain one and only\n"
       "one possible value)")
  .def("singleton_str", &tt::DomainBase::getStringSingleton, arg("self"),
       "Get singleton value as a string\n\n"
       "Raises:\n"
       "  access_error: self.is_singleton==False")
  .def("intersect", pure_virtual(&tt::DomainBase::intersect),
       args("self", "other"),
       "Check that the intersection between self and other is not empty.\n"
       "NOTE: often the intersection between domains with differents\n"
       "      types is empty.")
  .def("__eq__", pure_virtual(&tt::DomainBase::equals),
       args("self", "other"),
       "Test that self and other are identicals")
  .def("restrict", pure_virtual(&tt::DomainBase::restrictWith),
       return_internal_reference<>(),
       args("self", "other"),
       "Restrict self by intersecting it with other\n\n"
       "Raises:\n"
       "  empty_domain: The intersection with other is empty.\n\n"
       "Note: Intersecting domains with differnt type results on"
       "      an empty domain (including int vs float).")
  .def("as_tree", &tt::DomainBase::as_tree, arg("self"),
       "serialize self into an xml tree structure")
//  .def("build_tree", pure_virtual(&tt::DomainBase::build_tree),
//       arg("self"))
  .def("xml", &xml_str<tt::DomainBase>, arg("self"),
       "serialize self into an xml formatted string")
  .def("json", &json_str<tt::DomainBase>, arg("self"),
       "serialize self into an json formatted string")
  .def("__str__", pure_virtual(&str_impl<tt::DomainBase>), arg("self"),
       "serialize self into a human-readable string")
  ;
  
  class_<tt::DomainExcept, bases<tu::Exception> > dom_e
  ("domain_except", "Exceptions related to domains", no_init);
  
  s_py_err->attach<tt::DomainExcept>(dom_e.ptr());

  class_<tt::DomainAccess, bases<tt::DomainExcept> > acc_e
  ("access_error", "Invalid domain access error", no_init);
  
  s_py_err->attach<tt::DomainAccess>(acc_e.ptr());
  
  class_<tt::EmptyDomain, bases<tt::DomainExcept> > empty_e
  ("empty_domain", "Domain became empty", no_init);
  
  s_py_err->attach<tt::EmptyDomain>(empty_e.ptr());
  
  
  
  // class trex.domains.interval: trex.domains.domain
  //  abstract interval interface
  //    - has_lower() check if interval has a non infinite lower bound
  //    - has_upper() check if interval has a non infinite upper bound
  class_<interval_wrap, bases<tt::DomainBase>,
         boost::noncopyable>("interval", "Abstract trex interval",
                             init<tu::Symbol>(args("self", "type"),
                                              "create a full domain of type \"type\""))
  .def("has_lower", pure_virtual(&tt::BasicInterval::hasLower), arg("self"),
       "Check if lower bound is a value (as opposed to -inf)")
  .def("has_upper", pure_virtual(&tt::BasicInterval::hasUpper), arg("self"),
       "Check if upper bound is a value (as opposed to +inf)")
  .def("lower_str", &tt::BasicInterval::getStringLower, arg("self"),
       "Lower bound as a string\n")
  .def("upper_str", &tt::BasicInterval::getStringUpper, arg("self"),
       "Upper bound as a string\n")
  ;
  
  // class trex.domains.bool: trex.domains.interval
  //  boolean domain
  //   - __init__()     create a full boolean domain
  //   - __init__(bool) create a domain with the single value arg
  class_<tt::BooleanDomain, bases<tt::BasicInterval> >
  ("bool", "Boolean domain",
   init<>(arg("self"), "Create a full boolean domain "
          "(i.e. the domain accept both\n"
          "True and False)"))
  .def(init<bool>(args("self", "val"),
                  "Create a boolean domain limited to val"))
  ;
  
  // class trex.domains.int: trex.domains.interval
  //  integer domain
  //   - __init__()     create a full int domain
  //   - __init__(long) create a domain with the single value arg
  //   - __init__(long, long) create a domain with interval [arg1, arg2]
  class_<tt::IntegerDomain, bases<tt::BasicInterval> >
  ("int", "Integer domain",
   init<>(arg("self"), "Create the [-inf, +inf] int domain]"))
  .def(init<long>(args("self", "val"),
                  "Create the domain restricted to the constant val"))
  .def(init<long,long>(args("self", "lower_bound", "upper_bound"),
                       "Create the domain [lower_bound, upper_bound]"))
  ;

  // class trex.domains.float: trex.domains.interval
  //  float domain
  //   - __init__()     create a full int domain
  //   - __init__(double) create a domain with the single value arg
  //   - __init__(double, double) create a domain with interval [arg1, arg2]
  class_<tt::FloatDomain, bases<tt::BasicInterval> >
  ("float", "Float domain",
   init<>(arg("self"), "Create the [-inf, +inf] int domain]"))
  .def(init<double>(args("self", "val"),
                    "Create the domain restricted to the constant val"))
  .def(init<double,double>(args("self", "lower_bound", "upper_bound"),
                           "Create the domain [lower_bound, upper_bound]"))
  ;
  
  // class trex.domains.enumerated: trex.domains.domain
  // abstract enumerated domain interface
  //   - __len__()    number of elements
  class_<enum_wrap, bases<tt::DomainBase>, boost::noncopyable>
  ("enumerated", "Abstract trex Enumerated domain",
   init<tu::Symbol>(args("self", "type"),
                    "Create an enumerated domain of type \"type\""))
  .def("__len__", pure_virtual(&tt::BasicEnumerated::getSize),
       arg("self"),
       "Give the number of possible values in this domain.\n"
       "Note: A domain with a legnth of 0 is actually accepting all\n"
       "      possible values. Indeed, empty domains cannot exist in\n"
       "      trex and we decided to represent a full enumerated domain\n"
       "      by a domain with no constrained values as it allows to\n"
       "      represent domains for types that are not limited (such\n"
       "      as strings)")
  .def("elem_str", &get_elem_str,
       args("self", "index"),
       "Give the string representation of the element index of the domain.\n\n"
       "Raises:\n"
       "  access_error: index is out of range")
  // TODO need to implement __iter__
  ;
  
  
  // class trex.domains.string: trex.domains.enumerated
  //  string domain
  //    - __init__()           create the full domain
  //    - __init__(string)     create a domain with the single value arg
  //    - __init__(collection) create a domain with the elements given in colllection
  class_<tt::StringDomain, bases<tt::BasicEnumerated> >
  ("string", "string domain",
   init<>(arg("self"),
          "Create a new string domain that is full (i.e. accept\n"
          "any string)"))
  .def(init<std::string>(args("self", "val"),
                         "Create a new domain restricted to the single string val"))
  .def("__init__", &collection_init<tt::StringDomain, std::string>,
       "Create a string domain based on a collection of strings")
  ;

  
  // class trex.domains.enum: trex.domains.enumerated
  //  enum domain
  //    - __init__()                  create the full domain
  //    - __init__(trex.utils.symbol) create a domain with the single value arg
  //    - __init__(collection)        create a domain with the elements given in collection
  class_<tt::EnumDomain, bases<tt::BasicEnumerated> >
  ("enum", "enum domain",
   init<>(arg("self"),
          "Create a full enum domain"))
  .def(init<tu::Symbol>(args("self", "val"),
                        "Create an enum domain limited to the value val"))
  .def("__init__", &collection_init<tt::EnumDomain, tu::Symbol>,
       "Create an enum domain based on a collection of symbols")
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
  class_<tt::Variable>
  ("var",
   "Trex variable.\n"
   "A variable is a named object with an associated domain.\n"
   "The domain of a variable can only be restricted to less values\n"
   "and never be empty.",
   init<tu::Symbol, tt::DomainBase const &>
   (args("self", "name", "domain"),
    "Create the new variable \"name\" with the value \"domain\"\n\n"
    "Raises:\n"
    "  variable_exception: name is an empty string"))
  .add_property("name", make_function(&tt::Variable::name,
                                      return_internal_reference<>()),
                "Variable name")
  .add_property("domain", make_function(&tt::Variable::domain,
                                        return_internal_reference<>()),
                "Variable domain\n\n"
                "Raise:\n"
                "  variable_exception: domain of this variable is undefined")
  .def("from_xml", var_from_xml, (arg("xml")),
       "Create a variable from its xml description").staticmethod("from_xml")
  .def("restrict", restrict_domain, return_internal_reference<>(),
       args("self", "domain"),
       "Restrict the domain of this variable by intersecting it with domain\n\n"
       "Raises:\n"
       "  empty_domain: Intersection with domain is empty")
  .def("restrict", restrict_var, return_internal_reference<>(),
       args("self", "other"),
       "Restrict the domain of this variable by intersecting it with the\n"
       "domain of other\n\n"
       "Raises:\n"
       "  variable_exception: the 2 variables have different names\n"
       "  empty_domain: the intersection of the 2 domains is empty")
  .def("xml", &xml_str<tt::Variable>, arg("self"),
       "Serialize this variable as an xml formatted string")
  .def("json", &json_str<tt::Variable>, arg("self"),
       "Serialize this variable as a json fomratted string")
  .def("__str__", &str_impl<tt::Variable>, arg("self"),
       "Serialize this variable inot a human-readable string")
  ;

  class_<tt::VariableException, bases<tu::Exception> > var_e
  ("variable_exception", "Exception related to var", no_init);
  
  s_py_err->attach<tt::VariableException>(var_e.ptr());
  
}




