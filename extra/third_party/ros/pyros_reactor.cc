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
#include "pyros_reactor.hh"
#include <trex/python/python_thread.hh>
#include <boost/python/def.hpp>


using namespace TREX::ROS;
using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::python;
namespace bp=boost::python;
namespace bpt=boost::property_tree;

// structors

pyros_reactor::pyros_reactor(TeleoReactor::xml_arg_type arg)
:TeleoReactor(arg, false) {
  bpt::ptree::value_type const &node(xml_factory::node(arg));

  try {
//    syslog(log::info)<<"Loading trex into python";
//    m_trex = m_python->import("trex");
//    m_transaction = m_trex.attr("transaction");
//    m_domains = m_trex.attr("domains");
    
    // m_python->import("sys");
    
    bp::object main_module = m_python->import("__main__");
    bp::object main_ns = main_module.attr("__dict__");
    
    syslog(log::info)<<"Loading roslib";
    bp::exec("import roslib", main_ns);
    
//    m_roslib = m_python->import("roslib");
//    syslog(log::info)<<"Loading rospy";
//    m_rospy = m_python->import("rospy");
    
    
    syslog(log::info)<<"Creating my own environment (trex.__agents__."<<getAgentName()<<"."
      <<getName()<<")";
    
    bp::object trex_env;
    if( m_trex.attr("__dict__").contains("__agents__") )
      trex_env = m_trex.attr("__agents__");
    else {
      bp::scope cur(m_trex);
      trex_env = bp::object(bp::handle<>(bp::borrowed(PyImport_AddModule("__agents__"))));
    }
    if( trex_env.attr("__dict__").contains(getAgentName().c_str()) )
      trex_env = trex_env.attr(getAgentName().c_str());
    else {
      bp::scope cur(trex_env);
      trex_env = bp::object(bp::handle<>(bp::borrowed(PyImport_AddModule(getAgentName().c_str()))));
    }
    if( trex_env.attr("__dict__").contains(getName().c_str()) ) {
      syslog(log::warn)<<"environment trex.__agents__."<<getAgentName()<<"."<<getName()
        <<" already exist";
      m_env = trex_env.attr(getName().c_str());
    } else {
      bp::scope cur(trex_env);
      m_env = bp::object(bp::handle<>(bp::borrowed(PyImport_AddModule(getName().c_str()))));
    }
    
    size_t count = 0;
    for(bpt::ptree::const_iterator i=node.second.begin();
        node.second.end()!=i; ++i) {
      if( i->first=="Topic" ) {
        add_topic(*i);
        count += 1;
      }
    }
    if( 0==count )
      syslog(log::warn)<<"No connection created: this reactor is useless";
  } catch(bp::error_already_set const &e) {
    m_exc->unwrap_py_error();
  }
}

pyros_reactor::~pyros_reactor() {}


// modifiers

void pyros_reactor::add_topic(bpt::ptree::value_type const &desc) {
  Symbol tl_name = parse_attr<Symbol>(desc, "name");
  std::string ros_topic = parse_attr<std::string>(desc, "topic");
  boost::optional<std::string> py_type = parse_attr< boost::optional<std::string> >(desc, "type");
  bool write = parse_attr<bool>(false, desc, "accept_goals");
  
  if( !py_type )
    py_type = "genpy.message.Message";
  
  // First reserve the timeline
  provide(tl_name, write);
  if( isInternal(tl_name) ) {
    try {
      m_python->load_module_for(*py_type);
      std::string f_name(tl_name.str()+"_updated");
      {
        scoped_gil_release lock;
        bp::scope local(m_env);

        syslog()<<"Create python callback "<<f_name<<" for topic "<<ros_topic;
        bp::def(f_name.c_str(),
                bp::make_function(boost::bind(&pyros_reactor::ros_update,
                                              this, tl_name, _1),
                                  bp::default_call_policies(),
                                  boost::mpl::vector<void, bp::object>()));
        syslog()<<"Connect callback to topic";
        bp::exec("");
        
      }
    
    } catch(bp::error_already_set const &e) {
      m_exc->unwrap_py_error();
    }
  } else
    throw XmlError(desc, "Failed to declare \""+tl_name.str()+"\" as internal");
}


// callbacks

void pyros_reactor::handleInit() {
  
}

bool pyros_reactor::synchronize() {
  return true;
}

void pyros_reactor::ros_update(Symbol timeline, bp::object obj) {
  bp::dict attributes(obj.attr("__dict__"));
}

