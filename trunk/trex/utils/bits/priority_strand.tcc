/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Frederic Py.
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
#ifndef IN_trex_utils_priority_strand
# error "tcc files cannot be included outside of their corresponding header"
#else

namespace details {
  
  template<typename Ret>
  class future_task :public priority_strand::task {
  public:
    typedef boost::function<Ret ()>   fn_type;
    typedef boost::shared_future<Ret> future;
    
    
    template<typename Fn>
    explicit future_task(Fn f):m_function(f), m_set(false) {}
    template<typename Fn>
    future_task(Fn f, priority p):priority_strand::task(p), m_function(f) {}
    ~future_task() {
      if( !m_set ) {
        try {
          ERROR_CODE ec = make_error_code(ERRC::operation_canceled);
          m_result.set_exception(SYSTEM_ERROR(ec));
        } catch(...) {}
      }
    }
    
    future get_future() {
      return m_result.get_future();
    }
    
  private:
    fn_type              m_function;
    bool                 m_set;
    boost::promise<Ret>  m_result;
    
    void execute() {
      m_set = true;
      try {
        m_result.set_value(m_function());
      } catch(...) {
        m_result.set_exception(boost::current_exception());
      }
    }
    
    future_task() DELETED;
  }; // TREX::utils::details::future_task<>
  
  // Handle the special case where the function returns void
  template<>
  inline void future_task<void>::execute() {
    m_set = true;
    try {
      m_function();
      m_result.set_value();
    } catch(...) {
      m_result.set_exception(boost::current_exception());
    }
  }

  template<typename Ret>
  class async_task :public priority_strand::task {
  public:
    typedef boost::function<Ret ()>   fn_type;
    
    template<typename Fn>
    explicit async_task(Fn f):m_function(f) {}
    template<typename Fn>
    async_task(Fn f, priority p):priority_strand::task(p), m_function(f) {}
    ~async_task() {}
    
  private:
    fn_type              m_function;
    
    void execute() {
      try {
        m_function();
      } catch(...) {
        // Silently ignore for now ....
      }
    }
    
    async_task() DELETED;
  }; // TREX::utils::details::async_task<>
  
} // TREX::utils::details


template<typename Fn>
typename details::task_helper<Fn>::future priority_strand::post(Fn f) {
  typedef typename details::task_helper<Fn>::return_type return_type;
  details::future_task<return_type> *tsk(new details::future_task<return_type>(f));
  typename details::task_helper<Fn>::future ret(tsk->get_future());
  enqueue(tsk);
  return ret;
}

template<typename Fn>
typename details::task_helper<Fn>::future priority_strand::post(Fn f, priority_strand::priority_type p) {
  typedef typename details::task_helper<Fn>::return_type return_type;
  details::future_task<return_type> *tsk(new details::future_task<return_type>(f, p));
  typename details::task_helper<Fn>::future ret(tsk->get_future());
  enqueue(tsk);
  return ret;
}

template<typename Fn>
void priority_strand::send(Fn f) {
  typedef typename details::task_helper<Fn>::return_type return_type;
  enqueue(new details::async_task<return_type>(f));
}

template<typename Fn>
void priority_strand::send(Fn f, priority_strand::priority_type p) {
  typedef typename details::task_helper<Fn>::return_type return_type;
  enqueue(new details::async_task<return_type>(f, p));
}

#endif
