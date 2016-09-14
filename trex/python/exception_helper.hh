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
#ifndef H_trex_python_exception_helper
# define H_trex_python_exception_helper

# include <boost/python.hpp>
# include <boost/bind.hpp>

# include <trex/utils/SingletonUse.hh>
# include <trex/utils/id_mapper.hh>
# include <trex/utils/Exception.hh>


namespace TREX {
  namespace python {

    class exception_table;
    
    namespace details {
      
      class e_cvt_base:boost::noncopyable {
      public:
        typedef e_cvt_base base_type;
        typedef PyObject  *id_type;
        
        static id_type get_id(base_type const &me) {
          return me.m_py_type;
        }
        
        virtual ~e_cvt_base();
        
      protected:
        e_cvt_base(PyObject *type);
        virtual void convert(PyObject *except) =0;
        void  set_error(boost::python::object const &o);
        
        
      private:
        PyObject *m_py_type;
        
        friend class TREX::python::exception_table;
      };

      
      
      template<class Except>
      class except_cvt :public e_cvt_base {
      public:
        except_cvt(PyObject *type):e_cvt_base(type) {}
        ~except_cvt() {}
        
        void convert(Except const &e) {
          boost::python::object err(e);
          set_error(err);
        }
        
        void boost_init();
        
      private:
        void convert(PyObject *o) {
          Except err = boost::python::extract<Except>(o);
          throw err;
        }
      };
      
      template<class Except>
      class cvt_fn: public std::unary_function<Except, void> {
      public:
        explicit cvt_fn(except_cvt<Except> *ref):m_self(ref) {}
        
        void operator()(Except const &e) const {
          m_self->convert(e);
        }
        
      private:
        except_cvt<Except> *m_self;
      };
      
    } // TREX::python::details
    
    class python_error: public utils::Exception {
    public:
      python_error(python_error const &other) throw()
      :utils::Exception(other) {}
      ~python_error() throw() {}
      
    private:
      explicit python_error(std::string const &type) throw()
      :utils::Exception("Python error "+type) {}
      
      python_error(std::string const &type,
                   std::string const &what) throw()
      :utils::Exception("Python error "+type+": "+what) {}
      
      friend class exception_table;
    };
    
    class exception_table:boost::noncopyable {
    public:
      void unwrap_py_error();
      std::ostream &unwrap_py_error(std::ostream &out, bool rethrow=false);
      
      template<class Except>
      void attach(PyObject *type) {
        details::except_cvt<Except> *x = new details::except_cvt<Except>(type);
  
        if( add(x) ) {
          x->boost_init();
        } else
          delete x;
      }
      
    private:
      typedef utils::list_set< utils::pointer_id_traits<details::e_cvt_base> > exc_set;
      exc_set m_exceptions;
      boost::python::object m_traceback;
      
      exception_table();
      ~exception_table();
      
      bool add(details::e_cvt_base *x);
      
      friend class utils::SingletonWrapper<exception_table>;
    }; // TREX::python::details::exception_table
   
    
    template<class E>
    void details::except_cvt<E>::boost_init() {
      details::cvt_fn<E> cvt(this);
      boost::python::register_exception_translator<E>(cvt);
    }

    
  }
}

#endif