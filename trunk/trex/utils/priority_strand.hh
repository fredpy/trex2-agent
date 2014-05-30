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
    
      priority_strand(boost::asio::io_service &io);
      ~priority_strand();
      
      template<typename Fn>
      typename details::task_helper<Fn>::future post(Fn f);
      template<typename Fn>
      typename details::task_helper<Fn>::future post(Fn f, priority_type p);
      template<typename Fn>
      void async(Fn f);
      template<typename Fn>
      void async(Fn f, priority_type p);
      
    private:
      struct tsk_cmp {
        bool operator()(task *a, task *b) const;
      };

      typedef std::priority_queue<task *,std::vector<task *>, tsk_cmp> task_queue;

      boost::asio::strand m_strand;
      task_queue          m_tasks;
      
      
      void enqueue(task *tsk);
    
      void enqueue_sync(task *tsk);
      void dequeue_sync();
    }; // TREX::utils::priority_strand

# define IN_trex_utils_priority_strand
#  include "bits/priority_strand.tcc"
# undef IN_trex_utils_priority_strand

  } // TREX::utils
} // TREX

#endif // H_trex_utils_priority_strand