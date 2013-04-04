/** @file trex/utils/asio_runner.hh
 * @brief An helper to manage asio service execution
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */
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
#ifndef H_trex_utils_asio_runner
# define H_trex_utils_asio_runner

# include <boost/asio.hpp>
# include <boost/smart_ptr.hpp>
# include <boost/thread.hpp>

namespace trex {
  
  /** @brief Boost Asio service manager
   *
   * This class manages the creation and execution of and asio service
   * into threads. Threads are created as a group that is associated to
   * this service, it also maintain a work class so the threads do not
   * prematurely complete before its destruction.
   *
   * @note This class design assumes that there's only one instance of this
   * class running. Using multiple instances si possible although it will
   * result on at least as many thread as instances and the creation of new
   * threads will not ensure that the number of threads created do not exceeed
   * the hardware limitation. In that sense this class is more ment to be used
   * as a singleton.
   *
   * @author Frederic Py <fpy@mbari.org>
   * @ingroup utils
   */
  class asio_runner :boost::noncopyable {
  public:
    /** @brief Contructor
     *
     * Create a new instance with its corresponding service.
     *
     * @post a new asio service is running on a newly spawned thread
     */
    asio_runner();
    /** @brief Destructor
     *
     * This destructor will join all the threads this instance had created
     */
    ~asio_runner();
    
    /** @{
     * @brief Associated asio service
     *
     * Accessor to the io_service managed by this instance
     *
     * @return A reference to the service
     */
    boost::asio::io_service &service() {
      return m_io;
    }
    boost::asio::io_service &operator* () {
      return service();
    }
    boost::asio::io_service *operator->() {
      return &service();
    }
    /** @} */
    /** @brief Number of threads
     *
     * Indicate the number of threads that manage this service
     * @return The number of threads
     *
     * @sa thread_count(size_t, bool)
     */
    size_t thread_count() const {
      return m_threads.size();
    }
    /** @brief Increase the number of threads
     *
     * @param[in] n The desired minimum number of threads
     * @param[in] override_hw A flag used to allow to exceed the hardware
     *   concurrency
     *
     * This method allow user tho increase the number of threads spawned to
     * manage this service. It request the class to run at least @p n threads.
     *
     * If @p overide_hw is @c false (the default) then the system will adjust
     * @p n if it exceeeds the number of threads the system can physically run
     * (ie the number of cores on the host platform).
     *
     * The number of threads will also be adjusted only if @p n exceeds the current
     * number of threads.
     *
     * @return The number of threads associated to this service
     *
     * @sa thread_count() const
     */
    size_t thread_count(size_t n, bool override_hw=false);
    
  private:
    void spawn(size_t n_threads);
    
    boost::asio::io_service m_io;
    boost::scoped_ptr<boost::asio::io_service::work> m_active;
    boost::thread_group m_threads;
  }; // TREX::utils::asio_runner
  
} // trex

#endif // H_trex_utils_asio_runner
