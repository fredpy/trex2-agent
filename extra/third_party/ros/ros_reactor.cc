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
#include "ros_reactor.hh"
#include <trex/python/python_thread.hh>
#include <trex/domain/FloatDomain.hh>
#include <trex/domain/IntegerDomain.hh>


#include <boost/python/def.hpp>
#include <boost/python/stl_iterator.hpp>



using namespace TREX::ROS;
using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::python;
namespace bp=boost::python;
namespace bpt=boost::property_tree;

// structors

ros_reactor::ros_reactor(TeleoReactor::xml_arg_type arg)
:TeleoReactor(arg, false) {
  bpt::ptree::value_type const &node(xml_factory::node(arg));

  try {
    m_ros->strand().send(boost::bind(&ros_reactor::init_env, this),
                         roscpp_initializer::init_p);
    
    size_t count = 0;
    for(bpt::ptree::const_iterator i=node.second.begin();
        node.second.end()!=i; ++i) {
      if( i->first=="Topic" ) {
        m_ros->strand().send(boost::bind(&ros_reactor::add_topic, this, *i));
        count += 1;
      }
    }
    if( 0==count )
      syslog(log::warn)<<"No connection created: this reactor is useless";
  } catch(bp::error_already_set const &e) {
    m_exc->unwrap_py_error();
  }
}

ros_reactor::~ros_reactor() {}


// modifiers

void ros_reactor::add_topic(bpt::ptree::value_type const &desc) {
  Symbol tl_name = parse_attr<Symbol>(desc, "name");
  std::string ros_topic = parse_attr<std::string>(desc, "topic");
  boost::optional<std::string> py_type = parse_attr< boost::optional<std::string> >(desc, "type");
  bool write = parse_attr<bool>(false, desc, "accept_goals");
  
  if( !py_type )
    py_type = "genpy.message.Message";
  
  // First reserve the timeline
  provide(tl_name, write);
  if( isInternal(tl_name) ) {
    scoped_gil_release lock;
    try {
      bp::object type = m_python->load_module_for(*py_type);
      std::string f_name(tl_name.str()+"_updated");
      {
        bp::scope local(m_env);

        syslog()<<"Create python callback "<<f_name<<" for topic "<<ros_topic;
          bp::def(f_name.c_str(),
                  bp::make_function(boost::bind(&ros_reactor::ros_update,
                                                this, tl_name, _1),
                                    bp::default_call_policies(),
                                    boost::mpl::vector<void, bp::object>()));
        
        syslog()<<"Subscribe to topic "<<ros_topic;

        bp::object cb = m_env.attr(f_name.c_str());
        bp::object sub = m_ros->rospy().attr("Subscriber");
        
        m_env.attr((tl_name.str()+"_sub").c_str()) = sub(ros_topic.c_str(), type, cb);
        syslog()<<"done: "<<f_name<<"="
          <<bp::extract<char const *>(bp::str(m_env.attr((tl_name.str()+"_sub").c_str())));
      }
    
    } catch(bp::error_already_set const &e) {
      m_exc->unwrap_py_error();
    }
  } else
    throw XmlError(desc, "Failed to declare \""+tl_name.str()+"\" as internal");
}

// manipulators

void ros_reactor::init_env() {
  try {
    python::scoped_gil_release lock;
    m_trex = m_python->import("trex");
    m_domains = m_trex.attr("domains");
    m_transaction = m_trex.attr("transaction");
  
    syslog(log::info)<<"Creating python environment trex._agents_."
    <<getAgentName()<<"."<<getName();
  
    bp::object tmp;
    if( m_python->dir(m_trex).contains("_agents_") )
      tmp = m_trex.attr("_agents_");
    else {
      bp::scope cur(m_trex);
      tmp = bp::object(bp::handle<>(bp::borrowed(PyImport_AddModule("_agents_"))));
    }
  
    if( m_python->dir(tmp).contains(getAgentName().c_str()) )
      tmp = m_trex.attr(getAgentName().c_str());
    else {
      bp::scope cur(tmp);
      tmp = bp::object(bp::handle<>(bp::borrowed(PyImport_AddModule(getAgentName().c_str()))));
    }
  
    if( m_python->dir(tmp).contains(getName().c_str()) )
      throw transaction::ReactorException(*this,
                                          "trex._agents_."+getAgentName().str()+
                                          "."+getName().str()+" already exists");
    else {
      bp::scope cur(tmp);
      m_env = bp::object(bp::handle<>(bp::borrowed(PyImport_AddModule(getName().c_str()))));
    }
  } catch(bp::error_already_set const &e) {
    m_exc->unwrap_py_error();
  }
}

size_t ros_reactor::populate_attributes(Predicate &pred, std::string path,
                                        bp::object const &obj) {
  size_t ret = 0;

  for(bp::stl_input_iterator<bp::str> i(m_python->dir(obj)), end;
      end!=i; ++i) {
    if( !(i->startswith("_") || (*i)=="header" || (*i)=="child_frame_id") ) {
      bp::object const &attr = obj.attr(*i);
      if( !m_python->callable(attr) ) {
        std::string local = path;
        local += bp::extract<char const *>(*i);
        
        bp::extract<FloatDomain::base_type> is_float(attr);
        bp::extract<IntegerDomain::base_type> is_int(attr);
        
        if( is_float.check() ) {
          syslog()<<local<<" is float";
          if( is_int.check() )
            syslog()<<local<<" is int too *sigh*";
          pred.restrictAttribute(Variable(local, FloatDomain(is_float)));

          ++ret;
        } else if ( is_int.check() ) {
          syslog()<<local<<" is int";
          pred.restrictAttribute(Variable(local, IntegerDomain(is_int)));
          ++ret;
        } else {
          // TODO: check first if it is an obvious type: int, float, bool, ...
          // TODO:
        
          size_t sub = populate_attributes(pred, local+"_", attr);
      
          if( 0==sub ) {
            // This is not a structure
            syslog()<<pred.object()<<'.'<<pred.predicate()<<'.'<<path
            <<"="<<bp::extract<char const *>(bp::str(attr));
          } else
            ret += sub;
        }
      }
    }
  }
  return ret;
}


// callbacks

void ros_reactor::handleInit() {
  
}

bool ros_reactor::synchronize() {
  return true;
}

void ros_reactor::ros_update(Symbol timeline, bp::object obj) {
  python::scoped_gil_release lock;
  std::string pred;
  // Extract the type for predicate name
  if( m_python->dir(obj).contains("_type") ) {
    pred = bp::extract<char const *>(obj.attr("_type"));
    size_t pos = pred.find_last_of("./");
    if( std::string::npos!=pos ) {
      pred = pred.substr(pos+1);
    }
  }
  
  Observation obs(timeline, pred);
  populate_attributes(obs, "", obj);
  postObservation(obs);
}

