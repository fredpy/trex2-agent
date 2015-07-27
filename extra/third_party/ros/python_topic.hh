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
#ifndef H_trex_ros_python_topic
# define H_trex_ros_python_topic 

# include <trex/ros/bits/ros_timeline.hh>
# include <trex/python/python_env.hh>
# include <trex/python/exception_helper.hh>
# include <boost/property_tree/ptree.hpp>

namespace TREX {
  namespace ROS {
    
    class python_topic :public details::ros_timeline {
    public:
      using details::ros_timeline::xml_factory;
      using details::ros_timeline::xml_arg;
      
      python_topic(xml_arg arg);
      ~python_topic();
      
      std::string const &topic() const {
        return m_topic;
      }
      
    private:
      
      class path_alias {
      public:
        typedef boost::property_tree::path path_type;
        
        path_alias();
        explicit path_alias(path_type p);
        explicit path_alias(std::string const &name);
        ~path_alias();
        
        bool has_ros() const;
        path_type const   &ros() const {
          return m_ros;
        }
        std::string const &trex() const {
          return m_trex;
        }
        
        path_alias &operator /= (std::string const &attr);
        path_alias &operator /= (path_type other);
        bool operator < (path_alias const &other) const;
        bool operator == (path_alias const &other) const;
        
      private:
        path_type   m_ros;
        std::string m_trex;
        
      };

      
      size_t to_trex(transaction::Predicate &pred, path_alias path,
                     boost::python::object const &msg);
      size_t to_trex(transaction::Predicate &pred,
                     boost::python::object msg) {
        path_alias p;
        return to_trex(pred, p, msg);
      }
      
      bool set_ros(boost::python::object &msg, path_alias::path_type p,
                   boost::python::object val);
      size_t to_msg(transaction::Predicate const &pred,
                    boost::python::object &msg);
      
      void init_env();
      void init_topic();
      
      bool handle_request(transaction::goal_id g);
      void handle_recall(transaction::goal_id g);
      void synchronize(transaction::TICK date);
      
      void ros_cb(boost::python::object msg);

      std::string const m_topic;
      std::string const m_type_name;
      utils::SingletonUse<python::python_env>      m_python;
      utils::SingletonUse<python::exception_table> m_err;
      
      
      
      typedef std::multimap<path_alias, utils::Symbol> attr_map;
      attr_map m_attrs;
      
      bool add_attr(path_alias const &p, utils::Symbol const &type);
      
      
      boost::python::object m_env, m_trex,
        m_msg_type, m_ros_time, m_ros_duration;
    };
    
  }
}

#endif
