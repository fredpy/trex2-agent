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
#include "exception_helper.hh"

#include <boost/scope_exit.hpp>
#include <pyerrors.h>


using namespace TREX::python::details;
using namespace TREX::python;
namespace bp=boost::python;


/*
 * class TREX::python::details::exception_table
 */

// structors

exception_table::exception_table() {}

exception_table::~exception_table() {
  while( !m_exceptions.empty() ) {
    e_cvt_base *to_del = m_exceptions.front();
    m_exceptions.pop_front();
    delete to_del;
  }
}

// modifiers

bool exception_table::add(e_cvt_base *ref) {
  return m_exceptions.insert(ref).second;
}

// manipulators

void exception_table::unwrap_py_error() {
  if( PyErr_Occurred() ) {
    PyObject *exc,*val,*tb;
    
    // fetch error info
    PyErr_Fetch(&exc, &val, &tb);
    bp::handle<> hexc(exc), hval(bp::allow_null(val)), htb(bp::allow_null(tb));
    // clear error on python
    PyErr_Clear();
    
    
    if( NULL!=exc ) {
      // Look if I know the error
      exc_set::const_iterator pos = m_exceptions.find(exc);
      
      if( m_exceptions.end()!=pos && NULL!=val ) {
        // This type is known and has an instance -> convert it
        (*pos)->convert(val);
      } else {
        bp::object formatted_list, formatted;
        std::string type, msg;
        
        if( m_traceback.is_none() ) {
          m_traceback = bp::import("traceback");
        }
        // extract call stack info when available
        if( NULL==tb ) {
          bp::object fmt_exc_only(m_traceback.attr("format_exception_only"));
          formatted_list = fmt_exc_only(hexc, hval);
        } else {
          bp::object fmt_exc(m_traceback.attr("format_exception"));
          formatted_list = fmt_exc(hexc, hval, htb);
        }
        formatted = bp::str("\n").join(formatted_list);
        type = bp::extract<std::string>(bp::str(hexc));
        if( type.empty() )
          type = "<unknown>";
        msg = bp::extract<std::string>(formatted);
        if( !msg.empty() )
          type += ":\n"+msg;
        throw python_error(type);
      }
    } else {
    // send a python error with <null> type; but this should never occur
      throw python_error("<null>");
    }
  }
}

std::ostream &exception_table::unwrap_py_error(std::ostream &out, bool rethrow) {
  try {
    unwrap_py_error();
  } catch(std::exception const &e) {
    out<<e.what();
    if( rethrow )
      throw;
  } catch(...) {
    out<<"unknown exception";
    if( rethrow )
      throw;
  }
  return out;
}


/*
 * class TREX::python::details::e_cvt_base
 */

// structors

e_cvt_base::e_cvt_base(PyObject *type):m_py_type(type) {}

e_cvt_base::~e_cvt_base() {}

// manipulators

void e_cvt_base::set_error(boost::python::object const &o) {
  PyErr_SetObject(m_py_type, o.ptr());
}




