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
#ifndef H_trex_python_python_reactor
# define H_trex_python_python_reactor

# include <trex/transaction/TeleoReactor.hh>

# include <boost/python.hpp>

namespace TREX {
  namespace python {
    void log_error(boost::python::error_already_set const &e);

    class py_reactor;
    
    struct py_wrapper {
      py_wrapper(py_reactor *r):me(r) {}
      ~py_wrapper() {}
      
      py_reactor *me;
    };
    
    
    class reactor_proxy:boost::noncopyable {
    public:
      explicit reactor_proxy(py_wrapper const &r);
      virtual ~reactor_proxy();
      
      utils::Symbol const &name() const;
      transaction::TICK latency() const;
      transaction::TICK lookahead() const;
      transaction::TICK exec_latency() const;
      
      transaction::TICK initial() const;
      transaction::TICK final() const;
      transaction::TICK current() const;
      std::string date_str(transaction::TICK val) const;
      
      void use_tl(utils::Symbol const &tl, bool control);
      void provide_tl(utils::Symbol const &tl, bool control);
      bool is_internal(utils::Symbol const &tl) const;
      bool is_external(utils::Symbol const &tl) const;
      
      void post(transaction::Observation const &o, bool verbose);
      
      virtual void notify(transaction::Observation const &o) {}
      virtual bool synchronize() =0;
      
    private:
      py_reactor *m_impl;
      reactor_proxy();
    };
    
    class py_reactor :public transaction::TeleoReactor {
    public:
      py_reactor(xml_arg_type arg);
      ~py_reactor();
      
      void notify(transaction::Observation const &o);
      bool synchronize();
      
    private:
      boost::python::object m_self;
      
      friend class reactor_proxy;
    };
    
    class reactor_wrap:public reactor_proxy,
    public boost::python::wrapper<reactor_proxy> {
    public:
      explicit reactor_wrap(py_wrapper const &r);
      ~reactor_wrap();
      
      void notify(transaction::Observation const &o);
      void notify_default(transaction::Observation const &o);
      bool synchronize();
    };
    
    
    
    
    
//    class py_reactor;
//    
//    
//    class python
//    
//    
//    
//    class py_producer:public transaction::TeleoReactor::xml_factory::factory_type::producer {
//    public:
//      explicit py_producer(utils::Symbol const &name);
//      ~py_producer() {}
//      
//    private:
//      result_type produce(argument_type arg) const;
//    };
    
  } // TREX::python
} // TREX

#endif // H_trex_python_python_reactor
