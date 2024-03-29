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
#pragma once
#include <memory>

#include <boost/asio/io_service.hpp>
#include <boost/asio/strand.hpp>
#include <boost/tuple/tuple.hpp>

#include "bits/async_result.hh"
#include <system_error>

namespace TREX::utils {

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
class priority_strand  {
  class pimpl;

public:
  class task {
  public:
    typedef size_t priority;

    bool operator<(task const &other) const;

  protected:
    task() {}
    task(priority p) : m_level(p) {}
    virtual ~task() {}

    virtual void execute() = 0;

  private:
    std::optional<priority> m_level;

    friend class priority_strand;
  };

  typedef task::priority priority_type;
  typedef boost::asio::io_service::strand strand_type;

  /** @brief Constructor
   *
   * @param[in] io     An asio io_service
   * @param[in] active should the service starts right away
   *
   * Create a new instance the execution of the tasks will be
   * managed by @p io. If @p active is true (the default) then
   * this queue will be active right away otherwise it will stay
   * inactive until explicitely started
   */
  explicit priority_strand(boost::asio::io_service &io, bool active = true);
  priority_strand(priority_strand const &) =delete;
  explicit priority_strand(std::shared_ptr<strand_type> const &s,
                           bool active = true);
  /** @brief Detructor
   *
   * clear the queue of pending tasks for this instance and
   * destroy this instance.
   *
   * @dsa clear()
   */
  ~priority_strand();

  strand_type &strand();

  /** @brief Check if active
   *
   * Indiactes if thei instance is currently active or not. An
   * inactive queue enqueue the tasks to be executed but do not
   * process them until made active.
   *
   * @retval true if the queue is currently active
   * @retval false if the queue is currently inactive
   *
   * @sa start()
   * @sa stop()
   */
  bool is_active() const;

  bool completed() const;
  /** @brief Make the queue active
   *
   * Initiate the queue tasks processing loop. All tasks
   * previously inserted will be dequeues and executed until
   * the queue is ever emptied or topped
   *
   * @post the queue is active
   *
   * @sa bool is_active() const
   * @sa stop()
   * @sa clear()
   */
  void start();
  /** @brief Deactivate the queue
   *
   * Stops the queue tasks processing loop. Any tasks still in
   * the queue are put on hold untile the queue is either cleared
   * or started
   *
   * @post the queue is active
   *
   * @sa bool is_active() const
   * @sa start()
   * @sa clear()
   */
  void stop();

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
  template <typename Fn> typename details::task_helper<Fn>::future post(Fn f);
  template <typename Fn>
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
  template <typename Fn> void send(Fn f);
  template <typename Fn>
  void send(Fn f, typename details::task_helper<Fn>::handler handle);
  template <typename Fn> void send(Fn f, priority_type p);
  template <typename Fn>
  void send(Fn f, typename details::task_helper<Fn>::handler handle,
            priority_type p);
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
   * to a @c std::system_error exception with the error code
   * @c std::errc::operation_canceled
   */
  void clear();

private:
  std::shared_ptr<pimpl> m_impl;

  void enqueue(task *tsk);
}; // TREX::utils::priority_strand

#define IN_trex_utils_priority_strand
#include "bits/priority_strand.tcc"
#undef IN_trex_utils_priority_strand

} // namespace TREX::utils
