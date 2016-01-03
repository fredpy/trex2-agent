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
#ifndef H_trex_python_python_reactor
# define H_trex_python_python_reactor

# include <trex/transaction/TeleoReactor.hh>
# include "exception_helper.hh"
# include "python_env.hh"

# include <boost/python.hpp>

namespace TREX {
  namespace python {
    class py_reactor;
    
    struct py_wrapper {
      py_wrapper(py_reactor *r, boost::property_tree::ptree::value_type &node):me(r), xml_ref(node) {
      }
      ~py_wrapper() {}
      
      boost::property_tree::ptree::value_type const &xml() const {
        return xml_ref;
      }
      
      py_reactor *me;
      boost::property_tree::ptree::value_type &xml_ref;
    };
    
    
    class reactor_proxy:boost::noncopyable {
    public:
      explicit reactor_proxy(py_wrapper const &r);
      virtual ~reactor_proxy();
      
      bool is_verbose() const;
      void set_verbose(bool value);
      
      utils::Symbol const &name() const;
      utils::Symbol const &agent_name() const;
      transaction::graph const &graph() const;
      transaction::TICK latency() const;
      void set_latency(transaction::TICK val);
      transaction::TICK lookahead() const;
      void set_lookahead(transaction::TICK val);
      transaction::TICK exec_latency() const;
      
      transaction::TICK initial() const;
      transaction::TICK final() const;
      transaction::TICK current() const;
      std::string date_str(transaction::TICK val) const;
      
      double tick_duration() const;
      double as_seconds(transaction::TICK delta) const;
      
      void use_tl(utils::Symbol const &tl, bool control,
                  bool plan_listen);
      bool unuse_tl(utils::Symbol const &tl);
      void provide_tl(utils::Symbol const &tl, bool control,
                      bool plan_publish);
      bool unprovide_tl(utils::Symbol const &tl);
      bool is_internal(utils::Symbol const &tl) const;
      bool is_external(utils::Symbol const &tl) const;
      
      void post(transaction::Observation const &o, bool verbose);
      bool request(transaction::goal_id const &g);
      bool recall(transaction::goal_id const &g);
      bool post_plan(transaction::goal_id const &g);
      void cancel_plan(transaction::goal_id const &g);

      void info(std::string const &msg);
      void warning(std::string const &msg);
      void error(std::string const &msg);
      
      virtual void handle_init() {}
      virtual void handle_request(transaction::goal_id const &g) {}
      virtual void handle_recall(transaction::goal_id const &g) {}
      virtual void handle_new_tick() {}
      virtual void notify(transaction::Observation const &o) {}
      virtual bool synchronize() =0;
      virtual bool has_work();
      virtual void resume() {}
      virtual void new_plan(transaction::goal_id const &g) {}
      virtual void cancelled_plan(transaction::goal_id const &g) {}
      
      
    private:
      py_reactor *m_impl;
      reactor_proxy();
    };
    
    class reactor_wrap:public reactor_proxy,
    public boost::python::wrapper<reactor_proxy> {
    public:
      explicit reactor_wrap(py_wrapper const &r);
      ~reactor_wrap();
      
      void handle_init();
      void handle_init_default();
      void handle_request(transaction::goal_id const &g);
      void handle_request_default(transaction::goal_id const &g);
      void handle_recall(transaction::goal_id const &g);
      void handle_recall_default(transaction::goal_id const &g);
      void handle_new_tick();
      void handle_new_tick_default();
      void notify(transaction::Observation const &o);
      void notify_default(transaction::Observation const &o);
      bool synchronize();
      bool has_work();
      bool has_work_default();
      void resume();
      void resume_default();
      void new_plan(transaction::goal_id const &g);
      void new_plan_default(transaction::goal_id const &g);
      void cancelled_plan(transaction::goal_id const &g);
      void cancelled_plan_default(transaction::goal_id const &g);
    };
   
    class py_reactor :public transaction::TeleoReactor {
    public:
      py_reactor(xml_arg_type arg);
      ~py_reactor();
      
    private:
      reactor_proxy &self();
      
      void handleInit();
      void handleRequest(transaction::goal_id const &g);
      void handleRecall(transaction::goal_id const &g);
      void handleTickStart();
      void notify(transaction::Observation const &o);
      bool synchronize();
      bool hasWork();
      void resume();
      void newPlanToken(transaction::goal_id const &g);
      void cancelledPlanToken(transaction::goal_id const &g);
      
      boost::python::object m_obj;
      utils::SingletonUse<exception_table> m_exc;
      utils::SingletonUse<python_env> m_python;
      
      friend class reactor_proxy;
    };
    
    
    
    
  } // TREX::python
} // TREX

#endif // H_trex_python_python_reactor
