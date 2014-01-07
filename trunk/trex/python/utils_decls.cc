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
#include <trex/utils/Symbol.hh>
#include <trex/utils/log_manager.hh>
#include <trex/utils/XmlUtils.hh>

#include <boost/python.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/signals2/shared_connection_block.hpp>

// Need to indicate to spirit to be thread safe
#define BOOST_SPIRIT_THREADSAFE
#include <boost/property_tree/json_parser.hpp>


#include <functional>

using namespace boost::python;
namespace bp=boost::property_tree;

namespace {
  
  class scoped_gil_release:boost::noncopyable {
  public:
    scoped_gil_release() {
      m_gil_state = PyGILState_Ensure();
    }
    ~scoped_gil_release() {
      PyGILState_Release(m_gil_state);
    }
    
  private:
    PyGILState_STATE m_gil_state;
  };
  
  TREX::utils::singleton::use<TREX::utils::log_manager> s_log;

  class log_wrapper {
  public:
    explicit log_wrapper(std::string const &name)
    :m_name(name) {}
    explicit log_wrapper(TREX::utils::symbol const &name)
    :m_name(name) {}
    ~log_wrapper() {}
    
    
    std::string get_log_dir() const {
      return m_log->log_path().string();
    }
    void set_log_dir(std::string const &path) {
      m_log->log_path(path);
    }
    void log(TREX::utils::symbol const &what, std::string const &msg) {
      get_log_dir();
      m_log->syslog(m_name, what)<<msg;
    }
    void info(std::string const &msg) {
      log(TREX::utils::log::info, msg);
    }
    void warn(std::string const &msg) {
      log(TREX::utils::log::warn, msg);
    }
    void error(std::string const &msg) {
      log(TREX::utils::log::error, msg);
    }
    
    std::string use(std::string fname) {
      bool found;
      fname = m_log->use(fname, found).string();
      if( found )
        return fname;
      return std::string();
    }
        
    std::string path() const {
      get_log_dir();
      return m_log->search_path();
    }
    
    bool add_path(std::string dir) {
      return m_log->add_search_path(dir);
    }
    
    TREX::utils::symbol m_name;
  private:
    TREX::utils::singleton::use<TREX::utils::log_manager> m_log;
  };
  
  class py_log_handler {
  public:
    py_log_handler() {}
    virtual ~py_log_handler() {
      disconnect();
    }
    
    bool connected() const {
      return m_conn.connected();
    }
    void disconnect() {
      m_conn.disconnect();
    }
    
    virtual void new_entry(TREX::utils::log::entry::pointer e) =0;
    
  protected:
    void handle_interface(TREX::utils::log::entry::pointer e) {
      scoped_gil_release protect;
      boost::signals2::shared_connection_block lock(m_conn);
      try {
        new_entry(e);
      } catch(error_already_set const &e) {
        disconnect();
        // Extract error message for logging
        PyObject *ptype, *pvalue, *ptrace;
        PyErr_Fetch(&ptype, &pvalue, &ptrace);

        std::string msg = extract<std::string>(pvalue);
        
        m_log->syslog("_python_", TREX::utils::log::error)<<msg;
        // Reestablish error and display it in python
        PyErr_Restore(ptype, pvalue, ptrace);
        PyErr_Print();
      } catch(...) {
        disconnect();
        m_log->syslog("python", TREX::utils::log::error)<<"Unknown exception during python callback.";
      }
    }
    
    void init() {
      m_conn = m_log->on_new_log(boost::bind(&py_log_handler::handle_interface, this, _1),true);
    }
    
  private:
    TREX::utils::log::text_log::connection             m_conn;
    TREX::utils::singleton::use<TREX::utils::log_manager> m_log;
  };
  
  struct py_log_handler_wrap:py_log_handler, wrapper<py_log_handler> {
  public:
    py_log_handler_wrap() {
      py_log_handler::init();
    }
    
    void new_entry(TREX::utils::log::entry::pointer e);
  };
  
  
  bp::ptree xml_from_string(std::string const &str) {
    std::istringstream iss(str);
    bp::ptree ret;
    bp::read_xml(iss, ret, bp::xml_parser::no_comments|bp::xml_parser::trim_whitespace);
    return ret;
  }
  
  bp::ptree xml_from_file(std::string const &fname) {
    bp::ptree ret;
    bp::read_xml(fname, ret, bp::xml_parser::no_comments|bp::xml_parser::trim_whitespace);
    return ret;
  }

  bool recompact(bp::ptree::value_type &val, size_t indent=0) {
    if( val.second.empty() )
      return false;
    else {
      bp::ptree tmp;
      val.second.swap(tmp);
      std::string pad(' ', indent);
      
      if( tmp.size()==tmp.count("") ) {
        for(bp::ptree::iterator i=tmp.begin(); i!=tmp.end(); ++i) {
          recompact(*i, indent+1);
          val.second.add_child(val.first, i->second);
        }
        return true;
      } else {
        for(bp::ptree::iterator i=tmp.begin(); i!=tmp.end(); ++i) {
          if( recompact(*i, indent+1) ) {
            for(bp::ptree::iterator j=i->second.begin(); i->second.end()!=j;++j)
              val.second.add_child(i->first, j->second);
          } else {
            val.second.add_child(i->first, i->second);
          }
        }
        return false;
      }
    }
  }
  
  
  bp::ptree json_from_string(std::string const &str) {
    std::istringstream iss(str);
    bp::ptree ret;
    bp::read_json(iss, ret);
    bp::ptree::value_type tmp("", ret);
    recompact(tmp);
    
    return tmp.second;
  }
  
  std::string xml_to_string(bp::ptree const &p) {
    std::ostringstream oss;
    bp::write_xml(oss, p);
    return oss.str();
  }

  std::ostream &json_print(std::ostream &out, bp::ptree const &p, size_t indent=0,
                           bool attr=false, bool first=true);
  
  
  std::ostream &json_print(std::ostream &out, bp::ptree::value_type const &p, bool first, size_t indent) {
    if( p.first=="<xmlattr>" )
      return json_print(out, p.second, indent, true, first);
    else {
      std::string tab((indent+1), ' ');
      if( !first )
        out<<",\n";
      out<<tab<<'\"'<<p.first<<"\": ";
      json_print(out, p.second, indent+1);
      return out;
    }
  }
  
  std::ostream &json_print(std::ostream &out, bp::ptree const &p, size_t indent,
                           bool attr, bool first) {
    std::string tab(indent, ' ');
    
    if( p.empty() )
      out<<"\""<<p.data()<<"\"";
    else {
      if( !attr ) {
        if( indent>0 )
          out<<'\n';
        out<<tab<<"{\n";
      }
      for(bp::ptree::const_iterator i=p.begin(); p.end()!=i; ++i) {
        json_print(out, *i, first, indent);
        first = false;
      }
      if( !attr )
        out<<'\n'<<tab<<'}';
    }
    return out;
  }
  
  
//  void remove_attr(bp::ptree &p) {
//    for(bp::ptree::iterator i=p.begin(); p.end()!=i;) {
//      if( i->first=="<xmlattr>" ) {
//        bp::ptree tmp = i->second;
//        i = p.erase(i);
//        p.insert(i, tmp.begin(), tmp.end());
//      } else {
//        remove_attr(i->second);
//        ++i;
//      }
//    }
//  }
  
  std::string xml_to_json(bp::ptree const &p) {
    std::ostringstream oss;
//    bp::ptree compact = p;
//    // I need to recompact the <xmlattr>
//    remove_attr(compact);
//    bp::write_json(oss, compact);

    json_print(oss, p);
    return oss.str();
  }

  
  
  
  bool has_attribute(bp::ptree::value_type const &t, std::string const &name) {
    return TREX::utils::parse_attr< boost::optional<std::string> >(t, name);
  }
  
  std::string attribute(bp::ptree::value_type const &t, std::string const &name) {
    return TREX::utils::parse_attr<std::string>(t, name);
  }
  
  std::string symbol_rep(TREX::utils::symbol const &s) {
    return "<trex.utils.symbol '"+s.str()+"'>";
  }

}


// Python API for TREX::utils
//   this basic API only exposes the bare minimum. Namely:
//     - trex.utils.symbol      for TREX::utils::Symbol
//     - trex.utils.log         basic/barebone access to TREX::utils::LogManager
//     - trex.utils.log_handler a simple abstract class for handling of new log entries
//     - trex.utils.log_entry   the class used by trex for log entries 
void export_utils() {
  // Setup my submodule
  object module(handle<>(borrowed(PyImport_AddModule("trex.utils"))));
  scope().attr("utils") = module;
  scope my_scope = module;
  // from now on eveerything is under trex.utils

  docstring_options doc_options(true, true, false);
  
  module.attr("__doc__") = "Trex utility classes.\n"
  "The classes that are used by trex but are often general constructs that\n"
  "could be used otherwise. This often include general types such as symbols,\n"
  "xml tree, ...\n"
  "All of these objects are directly related to a class -- or set f classes --\n"
  "from TREXutils C++ library."
  ;
  
  // trex.utils.symbol class
  //   can be created with a string
  //   can be compared to each other with (==,!=,<,>,<=,=>)
  //   can be checked if empty()q
  //   supports str(s) and len(s)
  class_<TREX::utils::symbol>("symbol", "Unique instance symbolic value",
                              init<optional<std::string> >(args("name"),
                                                           "Create a new symbol with the given name"))
   .add_property("empty", &TREX::utils::symbol::empty,
        "Test if current instance is the empty symbol")
   .def("__len__", &TREX::utils::symbol::length, (arg("self")),
        "Length in character of the current instance")
   .def(self == self)
   .def(self != self)
   .def(self < self)
   .def(self > self)
   .def(self <= self)
   .def(self >= self)
   .def("__str__", &TREX::utils::symbol::str,
        return_value_policy<copy_const_reference>(),
        (arg("self")),
        "String representation. This just convert the symbol into a\n"
        "python string.")
  .def("__repr__", &symbol_rep,(arg("self")),
       "representation of the symbol. This just alter the usual python\n"
       "__repr__ in order to display the symbol value.")
  ;
  
    
  // python string can be implicitly converted into trex.symbol
  implicitly_convertible<std::string, TREX::utils::symbol>();
  
  
  // trex.utils.log_entry class
  // Log message entry
  //   - no constructor (produced internally on log messages)
  //   - is_dated : indicate if the entry has a date
  //   - date() : the date of the entry (if is_dated)
  //   - source() : the source of the entry (symbol)
  //   - kind() : the type of the entry (symbol, either "INFO", "WARN", ...)
  //   - content() : the message content as a string
  class_<TREX::utils::log::entry, TREX::utils::log::entry::pointer>("log_entry",
                                                                    "A single log entry message", no_init)
  .add_property("is_dated", &TREX::utils::log::entry::is_dated, "Check if dated")
  .add_property("date",
                make_function(&TREX::utils::log::entry::date, return_value_policy<copy_const_reference>()),
                "entry production date.\n"
                "valid only if is_dated is True"
                )
  .add_property("source",
                make_function(&TREX::utils::log::entry::source, return_internal_reference<>()),
                "entry producer name")
  .add_property("kind",
                make_function(&TREX::utils::log::entry::kind,return_internal_reference<>()),
                "entry type (such as info, warn, error,...)")
  .add_property("content",
                make_function(&TREX::utils::log::entry::content, return_value_policy<copy_const_reference>()),
                "entry message content"
                )
  ;

  // trex.utils.log class
  // simple log manager
  //   - constructor takes a symbol which will prefix any log messages produced by this class
  //   - name  attribute gives the symbol given at construction
  //   - dir   is a read/write attribute that indicates/sets the log directory
  //   - path  is a read only attribute that gives the trex search path
  //   - add_path adds the path passed as argument to the trex search path
  //   - use_file locates the file passed as argument in trex search path and return its path if found
  //   - info, wran, error produces the string passed as argument as a log message
  class_< log_wrapper, boost::shared_ptr<log_wrapper> >("log", "Log message producer for trex", init<TREX::utils::symbol>(args("self", "name"), "Create a new logger with the given source name"))
  .add_property("name", make_getter(&log_wrapper::m_name, return_internal_reference<>()),
                "Name of this log producer")
  .add_property("dir", &log_wrapper::get_log_dir, &log_wrapper::set_log_dir,
                "TREX log directory")
  .add_property("path", &log_wrapper::path,
                "TREX file search path")
  .def("use_file", &log_wrapper::use, (arg("self"), arg("file_name")),
       "Search for a file.\n"
       "This method locate the file file_name in TREX_PATH.\n"
       "If the file is found it copies it in the log directory and\n"
       "return its valid name. Otherwise it returns an empty string")
  .def("info", &log_wrapper::info, (arg("self"), arg("msg")),
       "Produces the log message msg into trex log as an info message")
  .def("warn", &log_wrapper::warn, (arg("self"), arg("msg")),
       "Produces the log message msg into trex log as a warning message")
  .def("error", &log_wrapper::error, (arg("self"), arg("msg")),
       "Produces the log message msg into trex log as an error message")
  .def("add_path", &log_wrapper::add_path, (arg("self"), arg("path")),
       "Add the directory path to TREX_PATH")
  ;
  
  // trex.utils.log_handler
  // new log entry handler abstract class
  //   abstract class that python user can derive to handle new log messages
  //   - connected    : indicate if the instance is still active
  //   - disconnect() : deactivate the handler, as of now I have no way to activate it
  //   - new_entry(e) : method that will be called by trex on a new log entry
  //                    this is the method that one can implement to handle log messages
  //                    in python
  class_< py_log_handler_wrap, boost::noncopyable>
  ("log_handler",
   "Logging entry handler.\n"
   "This class allow user to implement in python their own listener to trex \n"
   "log messages. Any declasses derived from this one will subscribe to new \n"
   "log messages events as soon as instantiated.\n\n"
   "Note:\n"
   "It is important to know that the new_entry method is called by trex in a\n"
   "different thread than the one where python is running. While trex do \n"
   "protect this call within C++, implementer of a new log_handler should \n"
   "make sure that the operations they do within this method are thread safe\n"
   "within python.",
   init<>(args("self"), "Default intializer.\n"))
  .add_property("connected", &py_log_handler::connected,
                "Checks if the handler is still active")
  .def("disconnect", &py_log_handler::disconnect, (arg("self")),
       "Disconnect this handler making it inactive. As fo today there's \n"
       "no way to reenable a disconnected handler.")
  .def("new_entry", pure_virtual(&py_log_handler::new_entry), (arg("self"), arg("entry")),
       "New entry callback.\n"
       "This method is called by trex whenever a new log entry has been \n"
       "produced. This method is pure virtual and therefore need to be \n"
       "redefined in a python derived class.\n"
       "Note also that this method is called in a different thread than \n"
       "the one where python is running: trex ensure basic thread safety\n"
       "with python interpreter and guarantee that this method is not \n"
       "called twice concurrently. Still implementer should consider \n"
       "that any operation done in this method can be concurrent to any\n"
       "python code and protect accordingly data shared with this\n"
       "method.")
  ;

  // Very simple classes to manipulate xml property trees
  
  class_<bp::ptree::value_type> tag("xml_tag",
                                    "XML configuration tree element.\n"
                                    "Represent an XML tag as a pair of its tag name and sub xml trees forest.\n"
                                    "Additionally ot provides utilities to ease the access to the tag\n"
                                    "attributes",
                                    no_init);
  tag.add_property("tag", make_getter(&bp::ptree::value_type::first),
          "Name of the tag")
  .def("has_attribute", &has_attribute, (arg("self"), arg("attr_name")),
       "Check if this tag have the attribute attr_name")
  .def("attribute", &attribute, (arg("self"), arg("attr_name")),
       "Extract the value of the tag attribute attr_name")
  ;

  class_<bp::ptree>("xml", "XML configuration tree", no_init)
  .def("from_str", &xml_from_string,
       (arg("xml_text")),
       "Parses the XML string xml_text to a new xml tree").staticmethod("from_str")
  .def("from_file", &xml_from_file,
       (arg("file_name")),
       "Parses the xml file name from_file into a new xml tree").staticmethod("from_file")
  .def("from_json", &json_from_string, (arg("json_text")),
       "Parse the JSON text json_text into an xml tree.\n"
       "while JSON is not XML the class we use in trex to parse xml\n"
       "is boost.property_tree. This function iexists just ofr the sake\n"
       "of testing JSON as an alternate to trex xml format").staticmethod("from_json")
  .add_property("content",
                make_function(static_cast<std::string const &(bp::ptree::*)() const>(&bp::ptree::data),
                              return_value_policy<copy_const_reference>()),
                "Give the tree text content if any")
  .def("__str__", &xml_to_string, (arg("self")),
       "String conversion. Display this instance in XML ")
  .def("__iter__", iterator<bp::ptree>(),
       "An iterator through the tree xml_tags")
  .def("__len__", &bp::ptree::size, (arg("self")), "Number of tags for this tree")
  .add_property("empty", &bp::ptree::empty, "Check if empty")
  .def("ext_file", &TREX::utils::ext_xml,
       (arg("self"), "attribute", arg("ahead")=true),
       "Inject external content.\n"
       "This method allow to inject the XML content of the file pointed\n"
       "by the given xml attribute into this tree. The ahead flag\n"
       "allow to inject the new nodes either at the beginning of this \n"
       "instance or toward its end. This function is used heavily in \n"
       "trex code to allow to distribute a mission configuration \n"
       "between multiple files.")
  .def("json", &xml_to_json, (arg("self")), "Display the tree as a JSON string")
  ;
  
  tag.add_property("forest", make_getter(&bp::ptree::value_type::second),
                   "allow to iterate on this tag sub elements.")
  ;
  
} // export_utils()



void py_log_handler_wrap::new_entry(TREX::utils::log::entry::pointer e) {
  this->get_override("new_entry")(e);
}

