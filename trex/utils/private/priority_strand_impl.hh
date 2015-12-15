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
#ifndef H_trex_utils_private_priority_stand_impl
# define H_trex_utils_private_priority_stand_impl

# include "../priority_strand.hh"

# include <boost/thread/mutex.hpp>
# include <boost/thread/shared_mutex.hpp>

# include <queue>

namespace trex {
  namespace utils {
    
    class priority_strand::pimpl: boost::noncopyable,
    public ENABLE_SHARED_FROM_THIS<priority_strand::pimpl> {
    public:
      typedef priority_strand::task *task_ref;
      
      pimpl(SHARED_PTR<boost::asio::strand> const &s);
      ~pimpl();
      
      boost::asio::strand &strand() {
        return *m_strand;
      }
      
      void enqueue(task_ref tsk);
      
      bool active() const;
      void start();
      void stop();
      
      size_t tasks() const;
      bool empty() const;
      void clear();
      
    private:
      struct task_cmp {
        bool operator()(task_ref a, task_ref b) const;
      };
      typedef std::priority_queue<task_ref, std::vector<task_ref>,
                                  task_cmp> task_queue;
      
      SHARED_PTR<boost::asio::strand> m_strand;
      mutable boost::shared_mutex     m_mutex;
      task_queue                      m_queue;
      bool                            m_active;
      
      
      void dequeue_sync();
      pimpl() DELETED;
    }; // TREX::utils::priority_strand::pimpl
    
  } // TREX::utils
} // TREX


#endif // H_trex_utils_private_priority_stand_impl