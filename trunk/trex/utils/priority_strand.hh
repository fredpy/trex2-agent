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
# include <boost/thread/shared_mutex.hpp>
# include <boost/utility/result_of.hpp>

# include <queue>

namespace TREX {
  namespace utils {
     
    namespace details {
      
      template<typename Fn>
      struct task_helper {
        typedef typename boost::result_of<Fn()>::type return_type;
        typedef boost::shared_future<return_type>     future;
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
      /** @brief Detructor */
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
       * @param[in] f The task to execute
       * @param[in] p A priority value
       *
       * Schedule @p f to be executed with the priority @p p. If @p p
       * is not provided then the tasl priority is set to the lowest
       * priority. The highest priority is 0 follewed by 1, ...
       *
       * This call return immediately with the task @p f being scheduled
       * for execution. 
       *
       * @note This method do not give any way to synchronize directly 
       * with the completion of @p f. If such synchronization is needed 
       * please uses @c priority_starnd::post instead
       *
       * @pre Fn is a functor with no argument (ie @p f() is valid)
       *
       * @sa priority_strand::post
       */
      template<typename Fn>
      void send(Fn f);
      template<typename Fn>
      void send(Fn f, priority_type p);
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