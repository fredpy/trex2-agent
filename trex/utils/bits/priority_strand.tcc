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
    explicit future_task(Fn f):m_function(f) {}
    template<typename Fn>
    future_task(Fn f, priority p):priority_strand::task(p), m_function(f) {}
    ~future_task() {}
    
    future get_future() {
      return m_result.get_future();
    }
    
  private:
    fn_type              m_function;
    boost::promise<Ret>  m_result;
    
    void execute() {
      try {
        m_result.set_value(m_function());
      } catch(...) {
        m_result.set_exception(boost::current_exception());
      }
    }
    
    future_task() DELETED;
  }; // TREX::utils::details::future_task<>
  
  template<>
  void future_task<void>::execute() {
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
void priority_strand::async(Fn f) {
  typedef typename details::task_helper<Fn>::return_type return_type;
  enqueue(new details::async_task<return_type>(f));
}

template<typename Fn>
void priority_strand::async(Fn f, priority_strand::priority_type p) {
  typedef typename details::task_helper<Fn>::return_type return_type;
  enqueue(new details::async_task<return_type>(f, p));
}

#endif
