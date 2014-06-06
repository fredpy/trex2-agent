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
#ifndef H_trex_utils_priority_strand
# define H_trex_utils_priority_strand

# include "platform/cpp11_deleted.hh"

# include <boost/asio/strand.hpp>
# include <boost/optional.hpp>
# include <boost/thread/future.hpp>
# include <boost/thread/mutex.hpp>
# include <boost/thread/shared_mutex.hpp>
# include <boost/utility/result_of.hpp>
# include <boost/tuple/tuple.hpp>

# include "platform/system_error.hh"

# include <queue>

namespace TREX {
  namespace utils {
    
    namespace details {
      
      template<typename Ret>
      class async_task;
      
    } // TREX::utils::details
   
    
#ifndef DOXYGEN
    
    template<typename Ret>
    class async_result;
    
  
    template<>
    class async_result<void> {
    public:
      ~async_result() {}
      
      bool is_exception() const {
        return bool(m_error);
      }
      boost::exception_ptr const &exception() const {
        return m_error;
      }
      void get() const {
        if( m_error )
          rethrow_exception(m_error);
      }
      void get(boost::promise<void> &p) const {
        if( m_error )
          p.set_exception(m_error);
        else
          p.set_value();
      }

    private:
      async_result() {}
      
      void set_exception(boost::exception_ptr const &e) {
        m_error = e;
      }
      
      void set(boost::function<void ()> const &fn) {
        try {
          fn();
        } catch(...) {
          set_exception(boost::current_exception());
        }
      }
      
      boost::exception_ptr m_error;
      
      friend details::async_task<void>;
      
      template<typename Ret>
      friend class async_result;
    }; // TREX::utils::async_result<void>
    
    
    template<typename Ret>
    class async_result {
    public:
      ~async_result() {}
      
      bool is_exception() const {
        return m_except.is_exception();
      }
      boost::exception_ptr const &exception() const {
        return m_except.exception();
      }
      Ret const &get() const {
        m_except.get();
        return *m_value;
      }
      void get(boost::promise<Ret> &p) const {
        if( is_exception() )
          p.set_exception(exception());
        else
          p.set_value(*m_value);
      }
      
    private:
      async_result() {}
      
      void set(boost::function<Ret ()> const &fn) {
        m_except.set(boost::bind(&async_result::set_value, this,
                                 boost::bind(fn)));
      }
      void set_value(Ret const &v) {
        m_value = v;
      }
      void set_exception(boost::exception_ptr const &e) {
        m_except.set_exception(e);
      }
      
      
      async_result<void>   m_except;
      boost::optional<Ret> m_value;
      
      friend details::async_task<Ret>;
    }; // TREX::utils::async_result<>
    
# else // DOXYGEN
    
    /** @brief Asynchronous task result placeholder
     *
     * This class is used by @c priority_strand to pass the 
     * result of the asynchronously executed task to a potential 
     * handling callback.
     *
     * The class itself present aninterface largely inspired from
     * boost::future or std::future from C++11. It can embed either
     * the retruned value from the asynchronous call or the exception 
     * that was produced by this call.
     *
     * @tparam Ty the return type of the function
     *
     * @relates priority_strand
     */
    template<typename Ty>
    class async_result {
    public:
      /** @brief Check if exception
       *
       * Check if this result is an excpetion
       *
       * @retval true if the result is an exception
       * @retval false otherwise
       */
      bool is_exception() const;
      /** @brief Get exception 
       *
       * If this instance holds an exception it returns a 
       * pointer to this exception or an empty pointer 
       * otherwise.
       *
       * @return A pointer to the exception if any
       * @sa is_exception() const
       */
      boost::exception_ptr const &exception() const;
      /** @brief Get outcome of the function
       *
       * This function reporduces the outcome of the
       * asynchronous fucntion. If the function threw 
       * an exception then this call will rethrow it 
       * otherwise it returns the value of the function 
       * if any.
       *
       * @note computed_type is convertible into Ty
       *
       * @throw an exception if is_excpetion() is true
       *
       * @return The returned value of the function
       * @sa is_exception() const
       */
      computed_type get() const;
      /** @brief transfert the value to a boost::promise
       *
       * @param[out] promise the promise to update
       *
       * Updates @p promise with the value or exception 
       * wrapped by this instance.
       */
      void get(boost::promise<Ty> &promise) const;
    };
    
# endif // DOXYGEN
    
    
    namespace details {
      
      template<typename Fn>
      struct task_helper {
        typedef typename boost::result_of<Fn()>::type return_type;
        typedef boost::shared_future<return_type>     future;
        
        typedef async_result<return_type>             wrapper;
      }; // TREX::utils::details::task_helper<>
      
    } // TREX::utils::details
    
    /** @brief Priority based strand
     *
     * This class implements an asynchronous task scheduller that will 
     * execute tasks based on their given priority. All the tasks are 
     * executed through the same strand ensuring they are not ran 
     * concurrently but the selection of the next task to be executed 
     * is based on a simple priority queue ensuring that when multiple
     * tasks are pending the one with the lowest priority value will 
     * be executed next.
     *
     * Tasks are functions with no argument and any returned type. They 
     * can be ever queued through @c post or @c send. The difference is that
     * while post return a future that allow the caller to gather the 
     * returned value, send provide no such synchronization mechanism 
     * and is meant for situation where the result of the task do not matter
     *
     * @author Frederic Py
     */
    class priority_strand {
    public:
      class task {
      public:
        typedef size_t priority;
        
        bool operator< (task const &other) const;
        
      protected:
        task() {}
        task(priority p):m_level(p) {}
        virtual ~task() {}
        
        virtual void execute() =0;
        
      private:
        boost::optional<priority> m_level;
        
        friend priority_strand;
      };
      
      typedef task::priority priority_type;
    
      /** @brief Constructor 
       *
       * @param[in] io An asio io_service
       *
       * Create a new instance the execution of the tasks will be 
       * managed by @p io
       */
      explicit priority_strand(boost::asio::io_service &io);
      /** @brief Detructor 
       *
       * clear the queue of pending tasks for this instance and 
       * destroy this instance.
       *
       * @dsa clear()
       */
      ~priority_strand();
      
      /** @{
       * @brief Post a task
       *
       * @tparam Fn A functor or function type
       *
       * @param[in] f The task to execute
       * @param[in] p A priority value
       *
       * Schedule @p f to be executed with the priority @p p. If @p p 
       * is not provided then the tasl priority is set to the lowest 
       * priority. The highest priority is 0 follewed by 1, ... 
       *
       * This call return immediately with the task @p f being scheduled 
       * for execution
       *
       * @pre Fn is a functor with no argument (ie @p f() is valid)
       * @return A future refering to the asynchronous result of @p f
       *
       * @sa priority_strand::send
       */
      template<typename Fn>
      typename details::task_helper<Fn>::future post(Fn f);
      template<typename Fn>
      typename details::task_helper<Fn>::future post(Fn f, priority_type p);
      /** @} */
      /** @{
       * @brief Send a task
       *
       * @tparam Fn A functor or function type
       *
       * @param[in] f      The task to execute
       * @param[in] handle A function to call on completion
       * @param[in] p      A priority value
       *
       * Schedule @p f to be executed with the priority @p p. If @p p
       * is not provided then the tasl priority is set to the lowest
       * priority. The highest priority is 0 followed by 1, ...
       *
       * This call return immediately with the task @p f being scheduled
       * for execution. If @p handle is provided it will be called after 
       * @p f is executed with its @c async_result as an argument.
       *
       *
       * @pre Fn is a functor with no argument (ie @p f() is valid)
       * @pre Handle is a functor of the form @c void(async_result<Ret> const &).
       * Where @c Ret is return type of @p Fn
       *
       * @sa priority_strand::post
       */
      template<typename Fn>
      void send(Fn f);
      template<typename Fn, typename Handler>
      void send(Fn f, Handler handle);
      template<typename Fn>
      void send(Fn f, priority_type p);
      template<typename Fn, typename Handler>
      void send(Fn f, Handler handle, priority_type p);
      /** @} */
      
      /** @brief Number of pending tasks
       *
       * @return  The number of tasks waiting un the queue
       *
       * @sa empty() const
       */
      size_t tasks() const;
      /** @brief Check if empty
       *
       * Test if this instance has no more task to execute
       *
       * @retval true if there's no task in the queue
       * @retval false otherwise
       *
       * @sa tasks() const
       */
      bool empty() const;
      
      /** @brief remove all pending tasks 
       *
       * Clear all the tasks that were either posted or sent but 
       * not yet executed. The result of all these tasks is set 
       * to a @c SYSTEM_ERROR exception with the error code
       * @c ERRC::operation_canceled
       */
      void clear();
      
    private:
      struct tsk_cmp {
        bool operator()(task *a, task *b) const;
      };

      typedef std::priority_queue<task *,std::vector<task *>, tsk_cmp> task_queue;

      
      boost::asio::strand m_strand;
      mutable boost::shared_mutex m_mutex;
      task_queue          m_tasks;      
      
      void enqueue(task *tsk);
      void dequeue_sync();
    }; // TREX::utils::priority_strand

# define IN_trex_utils_priority_strand
#  include "bits/priority_strand.tcc"
# undef IN_trex_utils_priority_strand

  } // TREX::utils
} // TREX

#endif // H_trex_utils_priority_strand