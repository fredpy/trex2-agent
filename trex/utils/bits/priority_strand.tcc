/* -*- C++ -*- */
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
  class async_task :public priority_strand::task {
  public:
    typedef boost::function<Ret ()>                 fn_type;
    typedef typename task_helper<fn_type>::wrapper  wrapper;
    
    template<typename Fn>
    async_task(Fn f):m_fn(f) {}
    template<typename Fn>
    async_task(Fn f, priority p):priority_strand::task(p), m_fn(f) {}
    virtual ~async_task() {
      handle  h;
      fn_type fn;
      boost::tie(fn, h) = get_task();
      if( fn && h ) {
        wrapper cancel;
        std::error_code ec = make_error_code(std::errc::operation_canceled);
        
        cancel.set_exception(boost::copy_exception(std::system_error(ec)));
        handling(h, cancel);
      }
    }
    
    template<typename Handle>
    void set_handler(Handle h) {
      boost::unique_lock<boost::mutex> lock(m_mutex);
      m_handler = h;
    }
    
  private:
    typedef boost::function<void (wrapper const &)> handle;
    
    boost::mutex  m_mutex;
    handle        m_handler;
    fn_type       m_fn;
  
    std::pair<fn_type, handle> get_task() {
      std::pair<fn_type, handle> ret;
      {
        boost::unique_lock<boost::mutex> lock(m_mutex);
        std::swap(ret.first, m_fn);
        std::swap(ret.second, m_handler);
      }
      return ret;
    }
    
    void execute() {
      handle  h;
      fn_type fn;
      boost::tie(fn, h) = get_task();
      if( fn ) {
        wrapper ret;
        ret.set(fn);
        if( h )
          handling(h, ret);
      }
    }
    
    static void handling(handle const &h, wrapper const &r) {
      try {
        h(r);
      } catch(...) {
        // silently ignore exceptions from handlers
      }
    }
    async_task() =delete;
  };
  
  
  /** @brief Asynchronous task attached to a promise/future
   *
   * This class is an asynchronous task for which its completion 
   * handler just set a boost::promise to the returned value.
   * This ia an utility class when one want to create an asynchronous 
   * call and wait for its result in a synchronous manner.
   *
   * @sa priority_strand::post
   */
  template<typename Ret>
  class future_task :public async_task<Ret> {
  public:
    using typename async_task<Ret>::fn_type;
    using typename async_task<Ret>::wrapper;
    using typename async_task<Ret>::priority;
    typedef typename task_helper<fn_type>::future future;
    
    template<typename Fn>
    future_task(Fn  f):async_task<Ret>(f) {
      init();
    }
    template<typename Fn>
    future_task(Fn  f, priority p):async_task<Ret>(f,p) {
      init();
    }
    
    future get_future() {
      return m_result.get_future().share();
    }
    
  private:
    boost::promise<Ret> m_result;
    
    void set(wrapper const &v) {
      v.get(m_result);
    }
    void init() {
      this->set_handler(boost::bind(&future_task::set, this, _1));
    }
  };

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
void priority_strand::send(Fn f,
                           typename details::task_helper<Fn>::handler handle) {
  typedef typename details::task_helper<Fn>::return_type return_type;
  details::async_task<return_type> *tsk;
  tsk = new details::async_task<return_type>(f);
  tsk->set_handler(handle);
  enqueue(tsk);
}


template<typename Fn>
void priority_strand::send(Fn f, priority_strand::priority_type p) {
  typedef typename details::task_helper<Fn>::return_type return_type;
  enqueue(new details::async_task<return_type>(f, p));
}

template<typename Fn>
void priority_strand::send(Fn f,
                           typename details::task_helper<Fn>::handler handle,
                           priority_strand::priority_type p) {
  typedef typename details::task_helper<Fn>::return_type return_type;
  details::async_task<return_type> *tsk;
  tsk = new details::async_task<return_type>(f, p);
  tsk->set_handler(handle);
  enqueue(tsk);
}

#endif
