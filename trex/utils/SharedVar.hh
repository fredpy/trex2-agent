/* -*- C++ -*- */
/** @file "SharedVar.hh"
 * @brief utilities for multi-thread shared variable
 *
 * This header declare classes and utilities to easily
 * share a variable between multiple threads.
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
#ifndef H_SharedVar
# define H_SharedVar

# include <boost/call_traits.hpp>
# include <boost/utility.hpp>
# include <boost/thread/recursive_mutex.hpp>

# include "Exception.hh"

namespace TREX {
  namespace utils {

    /** @brief Bad access exception
     * @relates SharedVar
     *
     * This exception is thrown by SharedVar when one tries
     * to access a SharedVar it has not locked.
     *
     * @author Frederic Py <fpy@mbari.org>
     *
     * @note The management of this error is more a coding issue.
     * It may be fine to be able to disable it when we are sure
     * that this condition is respected. (?)
     * @ingroup utils
     */
    class AccessExcept
      :public Exception {
    public:
      /** @brief Constructor.
       *
       * @param[in] message Error message.
       *
       * Create a new instance with associated message @p message
       */
      AccessExcept(std::string const &message) throw()
	:Exception(message) {}
      /** @brief Destructor. */
      ~AccessExcept() throw() {}
      
    }; // TREX::utils::AccessExcept
    
    /** @brief multi-thread shared variable
     *
     * This class implements a simple utility to manage a variable
     * which is manipulated by multiple threads. It ensures that this
     * variable is accessed only by the thread which have
     * reserved (locked) it.
     *
     * @tparam Ty Type of the variable to be shared
     *
     * @pre @p Ty must provides a copy constructor
     * @pre @p Ty must be default constructible
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    template<class Ty>
    class SharedVar :boost::noncopyable {
    private:
      typedef boost::recursive_timed_mutex mutex_type;

    public:
      /** @brief Scoped variable locking type
       *
       * This type cvan be used to locally lock one SharedVar based
       * in a C++ scope.
       * A new instance of this class will lock the mutex associated
       * to the SharedVar during its whiole lifetime and release the
       * mutex on its destruction.
       *
       * This provide a simple, exception safe, way to lock a
       * SharedVar during a critical section while avoiding to
       * have it remains locked in the eventuality of an exception.
       *
       * For example on can implement the function @c getval(SharedVar const &)
       * that returns the value of a SharedVar like this:
       * @code
       * template<class Ty>
       * Ty getval(SharedVar<Ty> const &var) {
       *   SharedVar<Ty>::scoped_lock slock(var);
       *   return *var;
       * }
       * @endcode
       * At the begining of this function the creation of @c slock locks
       * the mutex  associated to @c var. This alows to access to @c var
       * on the next line. Finally, as we leave the function context @c slock
       * will be destroyed and by doing so will release the mutex it locked during
       * its creation. 
       */
      typedef boost::unique_lock< SharedVar<Ty> > scoped_lock;
      
      /** @brief Constructor.
       *
       * @param[in] val An init value
       *
       * Create a new instance and set its value to @p val
       *
       * @post the newly created instance is not locked
       */
      SharedVar(typename boost::call_traits<Ty>::param_type val=Ty())
	:m_lockCount(0ul), m_var(val) {}
      /** @brief Destructor */
      ~SharedVar() {}
      
      /** @brief Locking method.
       *
       * This method locks the current variable.
       *
       * @note This call is blocking; If this inatnce is locked by
       * another thread. The caller will be suspended until we can
       * sucedsfully lock the varaible for this thread.
       *
       * @post The variable is locked by the current thread.
       *
       * @throw ErrnoExcept System error 
       *
       * @sa unlock() const
       */
      void lock() const {
	m_mtx.lock();
	++m_lockCount;
      }
      /** @brief Unlocking method.
       *
       * Unlock the mutex attached to this variable.
       *
       * @pre  The variable is locked by current thread.
       * @post The variable is not locked by this thread anymore.
       *
       * @throw AccessExcept Variable not locked by this thread.
       * @throw ErrnoExcept  System error.
       *
       * @sa lock() const
       * @sa ownIt() const
       */
      void unlock() const {
	if( ownIt() ) {
	  --m_lockCount;
	  m_mtx.unlock();
	}
      }
      
      
      /** @brief Assignment operator.
       *
       * @param[in] value A value
       * 
       * This methods set current instance to @p value. 
       *
       * @return @p value
       *
       * @note This call is thread safe and atomic. If current instance
       * is not  already locked by the calling thread, it will do so and
       * unlock it on completion.
       *
       * @throw ErrnoExcept System error.
       */
      typename boost::call_traits<Ty>::param_type 
      operator=(typename boost::call_traits<Ty>::param_type value) {
	scoped_lock lock(*this);
	m_var = value;
 	return value;
      }
      /** @brief Check for thread ownership
       *
       * This methods indicates if current instance is locked by
       * current thread.
       * A thread can manipulate a SharedVariable if and only if
       * this variable is locked by the thread. This method allows
       * user to check such information.
       *
       * @retval true  If this instance is locked by the calliung thread.
       * @retval false otherwise
       *
       * @sa lock() const
       * @sa unlock() const
       */
      bool ownIt() const {
	mutex_type::scoped_try_lock lock(m_mtx);
	return lock.owns_lock() && m_lockCount>0;
      }
      
      /** @brief Dereferencing operator
       *
       * Give access to the shared variable
       *
       * @pre This instance is locked by current thread
       *
       * @return A reference to the variable
       *
       * @throw AccessExcept Variable is not locked by current thread.
       * @throw ErrnoExcept  System error
       *
       * @sa lock() const
       * @sa ownIt() const
       * @sa operator->()
       * @{
       */
      Ty &operator* () {
	if( !ownIt() )
	  throw AccessExcept("Cannot access a shared var which I don't own.");
	return m_var;
      }

      Ty const &operator* () const {
	if( !ownIt() )
	  throw AccessExcept("Cannot access a shared var which I don't own.");
	return m_var;
      }
      /** @} */
      
      /** @brief Access operator.
       * 
       * Give access to variable and more specifically to its attributes.
       *
       * @pre This instance is locked by current thread
       *
       * @return A pointer to the variable allowing access to its attributes
       *
       * @throw AccessExcept Variable is not owned by current thread.
       * @throw ErrnoExcept  System error.
       *
       * @sa lock() const
       * @sa ownIt() const
       * @sa operator* ()
       * @{
       */
      Ty *operator->() {
	return &operator* ();
      }

      Ty const *operator->() const {
	return &operator* ();
      }
      /** @} */
      
    private:
      /** @brief Mutex to lock/unlock variable */ 
      mutable mutex_type m_mtx;       //!< @brief System mutex for this variable
      /** @brief mutex lock counter
       *
       * This attribute count the number of locks currently active on
       * this instance. By design the counter will be modified only by
       * the thread which is currently locking the variable.
       *
       * The variable is not locked by any thread as soon as this counter reaches 0.
       * 
       * @sa lock() const
       * @sa unlock() const
       */
      mutable size_t     m_lockCount; 
      
      /** @brief Variable value */
      Ty m_var;
      
    }; // TREX::utils::SharedVar<>

  } // TREX::utils
} // TREX

#endif // H_SharedVar
