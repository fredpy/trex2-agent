/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2011, MBARI.
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
#include "asio_runner.hh"
#include "cpu_clock.hh"
#include "chrono_helper.hh"

using namespace TREX::utils;
using namespace boost::asio;

# ifdef BOOST_ASIO_ENABLE_HANDLER_TRACKING
# include "asio_fstream.hh"

namespace {
  
#if !defined(CPP11_HAS_CHRONO) && defined(BOOST_CHRONO_HAS_HREAD_CLOCK)
  typedef CHRONO::thread_clock perf_clock;
#else
  typedef cpu_clock perf_clock;
#endif
  
  typedef CHRONO::high_resolution_clock rt_clock;


  class timing_thread_log {
  public:
    timing_thread_log(std::string const &fname)
    :m_file(m_service),
    m_thread(boost::bind(&io_service::run, &m_service)) {
      m_file.open(fname);
    }
    ~timing_thread_log() {
      m_thread.join();
    }
    
    async_ofstream &out() {
      return m_file;
    }
    async_ofstream &operator *() {
      return out();
    }
  private:
    io_service m_service;
    async_ofstream    m_file;
    boost::thread    m_thread;
  };
  
  timing_thread_log io_log("trex_asio.log");
  
}

# endif 

/*
 * class TREX::utils::asio_runner
 */

// structors 

asio_runner::asio_runner() {
  // Create a work for maintaining our service 
  m_active.reset(new io_service::work(m_io));
}

asio_runner::asio_runner(size_t n_threads) {
  m_active.reset(new io_service::work(m_io));
  if( n_threads>0 )
    thread_count(n_threads);
}


asio_runner::~asio_runner() {
  // complete our work so threads can complete
  m_active.reset();
  // wait for our threads to join:
  //  can take time if they still have tasks to do
  m_threads.join_all();
}

// manipulators

size_t asio_runner::thread_count(size_t n, bool override) {  
  // correct the number of threads
  if( !override ) {
    // Max thread is number of cores minus the main thread
    size_t max_count = 2*boost::thread::hardware_concurrency();
    
    if( n > max_count )
      n=max_count;
  }

  size_t cur = thread_count();
  if( cur < n ) {
    // spawn the number of threads needed to reach n
    spawn(n-cur);
    return thread_count();
  }
  return cur;
}

void asio_runner::spawn(size_t n) {
  for(size_t i=0; i<n; ++i) 
    m_threads.create_thread(boost::bind(&asio_runner::thread_task, this));
}

# undef CHECK_INTERRUPTED
#if (BOOST_VERSION < 104700)
# define CHECK_INTERRUPTED
# warning "Your boost version is deprecated. Update to 1.47 or later"
#endif // BOOST_VERSION

void asio_runner::thread_task() {
#ifdef CHECK_INTERRUPTED
  bool interrupted;
  do {
    interrupted = false;
#else
  do {
#endif // CHECK_INTERRUPTED
    try {
#ifdef BOOST_ASIO_ENABLE_HANDLER_TRACKING
      size_t work_tasks;
      perf_clock::duration cpu_time;
      rt_clock::duration   real_time;
  
      do {
        {
          chronograph<perf_clock> perf(cpu_time);
          chronograph<rt_clock>   real(real_time);
        
          work_tasks = m_io.poll_one();
        }
        if( work_tasks ) {
          (*io_log)<<rt_clock::now().time_since_epoch().count()
                  <<", "<<boost::this_thread::get_id()
                  <<", "<<cpu_time.count()<<", "<<real_time.count()
                  <<", "<<work_tasks<<std::endl;
        }
        
        // Let the thread slow down a bit
        boost::this_thread::yield();
      } while( work_tasks>0 );
#else  // !BOOST_ASIO_ENABLE_HANDLER_TRACKING

      m_io.run();

#endif // BOOST_ASIO_ENABLE_HANDLER_TRACKING
    } catch(...) {
#ifdef CHECK_INTERRUPTED
      interrupted = true;
    }
  } while( !interrupted ); // before 1.47 there is no way to check for
                           // io service being stopped. Instead we use
                           // 'interrupted' local var to identify when
                           // an exception was thrown.
#else // !CHECK_INTERRUPTED
    }
  } while( !m_io.stopped() ); // boost >=1.47 allow to test if the io 
                              // service has been stopped. We use it
                              // to identify the end of the thread
#endif // CHECK_INTERRUPTED
}



