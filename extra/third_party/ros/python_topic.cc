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
#include "python_topic.hh"
#include "ros_reactor.hh"

#include <trex/python/python_thread.hh>
#include <trex/domain/StringDomain.hh>
#include <trex/domain/FloatDomain.hh>
#include <trex/domain/BooleanDomain.hh>
#include <trex/domain/IntegerDomain.hh>

#include <boost/python/stl_iterator.hpp>


using namespace TREX::ROS;
using namespace TREX::utils;
using namespace TREX::python;
using namespace TREX::transaction;

namespace py=boost::python;

namespace {
  
  ros_reactor::tl_factory::declare<python_topic> decl("Topic");
  
}

/*
 * class TREX::ROS::python_topic::path_alias
 */

// structors

python_topic::path_alias::path_alias() {}

python_topic::path_alias::path_alias(path_type p):m_ros(p) {
  std::ostringstream oss;
  
  while( !p.empty() ) {
    oss<<p.reduce();
    if( !p.empty() )
      oss.put('_');
  }
  m_trex = oss.str();
}

python_topic::path_alias::path_alias(std::string const &name)
:m_trex(name) {}

python_topic::path_alias::~path_alias() {}

bool python_topic::path_alias::has_ros() const {
  return ( !m_ros.empty() )|| m_trex.empty();
}

bool python_topic::path_alias::operator < (path_alias const &other) const {
  return m_trex < other.trex();
}

bool python_topic::path_alias::operator == (path_alias const &other) const {
  path_type a=m_ros, b=other.ros();
  
  while( !(a.empty() || b.empty()) ) {
    std::string a_attr(a.reduce()), b_attr(b.reduce());
    if( a_attr!=b_attr )
      return false;
  }
  return a.empty() && b.empty();
}

python_topic::path_alias &python_topic::path_alias::operator /= (std::string const &attr) {
  if( m_trex.empty() ) {
    m_trex = attr;
    m_ros /= attr;
  } else {
    m_trex += "_" + attr;
    if( !m_ros.empty() )
      m_ros /= attr;
  }
  return *this;
}

python_topic::path_alias &python_topic::path_alias::operator /= (path_type p) {
  while( !p.empty() ) {
    operator /=(p.reduce());
  }
  return *this;
}



/*
 * class TREX::ROS::python_topic
 */

// structors

python_topic::python_topic(python_topic::xml_arg arg)
:details::ros_timeline(arg, parse_attr<bool>(false, xml_factory::node(arg),
                                             "control")),
m_topic(parse_attr<std::string>(xml_factory::node(arg), "topic")),
m_type_name(parse_attr<std::string>(xml_factory::node(arg), "type")),
m_pred("Holds") {
  strand().send(boost::bind(&python_topic::init_env, this),
                roscpp_initializer::init_p);
  strand().send(boost::bind(&python_topic::init_topic, this),
                roscpp_initializer::init_p);
}

python_topic::~python_topic() {
  
}

// manipulators

void python_topic::init_env() {
  syslog(log::info)<<"init_env";
  try {
    scoped_gil_release lock;
    py::object tmp;
    
    m_trex = m_python->import("trex");
    tmp = m_python->add_module(m_trex, "_agents_");
    tmp = m_python->add_module(tmp, reactor().getAgentName().c_str());
    m_env = m_python->add_module(tmp, reactor().getName().c_str());
    
    m_ros_time = reactor().rospy().attr("Time");
    m_ros_duration = reactor().rospy().attr("Duration");
    
  } catch(py::error_already_set const &e) {
    m_err->unwrap_py_error();
  }
}

void python_topic::init_topic() {
  try {
    scoped_gil_release lock;
    std::string f_name(name().str()+"_updated");
    
    m_msg_type = m_python->load_module_for(m_type_name);
    if( m_python->dir(m_msg_type).contains("_type") ) {
      m_pred = py::extract<char const *>(m_msg_type.attr("_type"));
      size_t pos = m_pred.find_last_of("./");
      if( std::string::npos!=pos )
        m_pred = m_pred.substr(pos+1);
      syslog(log::info)<<"Predicate name identified as "<<m_pred;
    } else {
      syslog(log::warn)<<"Type "<<m_type_name<<" does not have a _type attribute.\n"
        <<"predicates will be published as "<<m_pred<<" instead";
    }
    
    if( controllable() ) {
      py::object tmp = m_msg_type();
      syslog()<<"Creating serialization template for "<<m_type_name;
      Observation foo = new_obs(m_pred);
      to_trex(foo, tmp);
    }

    // TODO:: examine my type before creating the callback
    
    py::object cb;
    {
      py::scope local(m_env);
      syslog()<<"Create python callback "<<f_name<<" for topic "<<m_topic;
      py::def(f_name.c_str(),
              py::make_function(boost::bind(&python_topic::ros_cb, this, _1),
                                py::default_call_policies(),
                                boost::mpl::vector<void, py::object>()));
      
      cb = m_env.attr(f_name.c_str());
    }
    
    py::object sub_class = reactor().rospy().attr("Subscriber");
    m_env.attr((name().str()+"_sub").c_str()) = sub_class(m_topic.c_str(),
                                                          m_msg_type, cb);
    
    
    syslog()<<"Added topic subscription as "<< (name().str()+"_sub");
  } catch(py::error_already_set const &e) {
    m_err->unwrap_py_error();
  }
}

bool python_topic::add_attr(python_topic::path_alias const &p,
                            Symbol const &type) {
  attr_map::iterator from, to;
  for(boost::tie(from, to) = m_attrs.equal_range(p); from!=to; ++from)
    if( from->first==p )
      return false;
  m_attrs.insert(attr_map::value_type(p, type));
  return true;
}


size_t python_topic::to_trex(transaction::Predicate &pred, path_alias path,
                             py::object const &msg) {
  size_t ret = 0;
  
  
  for(py::stl_input_iterator<py::str> i(m_python->dir(msg)), end;
      end!=i; ++i) {
    path_alias local(path);
    
    if( !( i->startswith("_") || "header"==(*i) || "child_frame_id"==(*i) ) ) {
      py::object attr = msg.attr(*i);
      
      if( !m_python->callable(attr) ) {
        local /=  std::string(py::extract<char const *>(*i));
        
        // Now check if it is basic type
        if( PyString_Check(attr.ptr()) ) {
          // It is a str
          std::string val((py::extract<char const *>(attr)));
          pred.restrictAttribute(local.trex(), StringDomain(val));
          add_attr(local, StringDomain::type_name);
          ++ret;
        } else {
          py::extract<FloatDomain::base_type> is_float(attr);
          
          if( is_float.check() ) {
            pred.restrictAttribute(local.trex(),
                                   FloatDomain(is_float));
            add_attr(local, FloatDomain::type_name);
            ++ret;
          } else {
            py::extract<IntegerDomain::base_type> is_int(attr);
            
            if( is_int.check() ) {
              py::extract<bool> is_bool(attr);
              
              if( is_bool.check() ) {
                pred.restrictAttribute(local.trex(),
                                       BooleanDomain(is_bool));
                add_attr(local, BooleanDomain::type_name);
              } else {
                pred.restrictAttribute(local.trex(),
                                       IntegerDomain(is_int));
                add_attr(local, IntegerDomain::type_name);
              }
              ++ret;
            } else if( m_python->is_instance(attr, m_ros_time) ) {
              long double value = py::extract<long double>(attr.attr("to_time")()),
                now = py::extract<long double>(m_ros_time.attr("now")().attr("to_time")());
              TICK date = reactor().getCurrentTick();
              CHRONO::duration<long double> tmp(value-now);
              ros_reactor::duration_type
                delta = CHRONO::duration_cast<ros_reactor::duration_type>(tmp);
              
              // TODO: ensure that tick is properly rounded
              date += delta.count()/reactor().tickDuration().count();
              pred.restrictAttribute(local.trex(),
                                     IntegerDomain(date));
              add_attr(local, "date");
              ++ret;
            } else if( m_python->is_instance(attr, m_ros_duration) ) {
              CHRONO::duration<long double> val(py::extract<long double>(attr.attr("to_sec")()));
              ros_reactor::duration_type dur = CHRONO::duration_cast<ros_reactor::duration_type>(val);
              TICK value = dur.count()/reactor().tickDuration().count();
              // TODO: ensure that duration is properly set to the floor
              pred.restrictAttribute(local.trex(),
                                     IntegerDomain(value));
              add_attr(local, "duration");
              ++ret;
            } else {
              // Try to see if I can recurse deeper on the structure
              size_t sub = to_trex(pred, local, attr);
              
              if( 0==sub ) {
//                syslog(log::warn)<<"Not able to convert to trex: "<<pred.object()
//                  <<'.'<<pred.predicate()<<'.'<<path.trex()
//                  <<"="<<py::extract<char const *>(py::str(attr));
              } else
                ret += sub;
            }
          }
        }
      }
    }
  }
  return ret;
}

bool python_topic::set_ros(py::object &msg,
                           python_topic::path_alias::path_type p,
                           py::object val) {
  // TODO: do some basic type checking
  if( p.empty() )
    msg = val;
  else {
    py::object tmp = msg;
    while( !p.single() )
      tmp = tmp.attr(p.reduce().c_str());
    tmp.attr(p.reduce().c_str()) = val;
  }
  return true;
}


size_t python_topic::to_msg(Predicate const &pred, boost::python::object &msg) {
  size_t count = 0;
  for(attr_map::const_iterator i=m_attrs.begin(); m_attrs.end()!=i; ++i) {
    if( pred.hasAttribute(i->first.trex()) ) {
      DomainBase const &dom = pred[i->first.trex()].domain();
      
      
      if( !dom.isSingleton() )
        syslog(log::warn)<<"Domain for "<<i->first.trex()<<"="<<dom<<" is not a singleton";
      else {
        py::object val;
        
        if( i->second=="date" ) {
          TICK trex_date = dom.getTypedSingleton<IntegerDomain::base_type, false>(),
            trex_now = reactor().getCurrentTick();
          long double now = py::extract<long double>(m_ros_time.attr("now")().attr("to_time")());
          graph::duration_type delta = reactor().tickDuration() * (trex_date-trex_now);
          
          now += CHRONO::duration_cast< CHRONO::duration<long double> >(delta).count();
          val = m_ros_time(now);
        } else if( i->second=="duration" ) {
          TICK trex_dur = dom.getTypedSingleton<IntegerDomain::base_type, false>();
          graph::duration_type rt_dur = reactor().tickDuration()*trex_dur;
          long double ros_dur = CHRONO::duration_cast< CHRONO::duration<long double> >(rt_dur).count();
      
          val = m_ros_duration(ros_dur);
        } else if( i->second==IntegerDomain::type_name ) {
          val = py::object(dom.getTypedSingleton<IntegerDomain::base_type, false>());
        } else if( i->second==FloatDomain::type_name ) {
          val = py::object(dom.getTypedSingleton<FloatDomain::base_type, false>());
        } else if( i->second==BooleanDomain::type_name ) {
          val = py::object(dom.getTypedSingleton<bool, false>());
        } else if( i->second==StringDomain::type_name ) {
          val = py::str(dom.getStringSingleton().c_str());
        }
        if( set_ros(msg, i->first.ros(), val) )
          ++count;
      }
    }
  }
  return count;
}




// callbacks

void python_topic::ros_cb(py::object msg) {
  try {
    scoped_gil_release lock;
    Observation obs = new_obs(m_pred);
    to_trex(obs,msg);
    notify(obs);
  } catch(py::error_already_set e) {
    m_err->unwrap_py_error();
  }
}

bool python_topic::handle_request(goal_id g) {
  syslog(log::warn)<<"Python ROS Topic goal handling not yet implemented";
  return false;
}

void python_topic::handle_recall(goal_id g) {
  syslog(log::warn)<<"Python ROS Topic goal handling not yet implemented";
}

void python_topic::synchronize(TICK date) {
  
}




