/* -*- C++ -*-
 * $Id$
 */
/** @file "TextLog.hh"
 * @brief Definition of the TextLog class
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
#ifndef _TEXTLOG_HH
# define _TEXTLOG_HH

# include <fstream>
# include <memory>
# include <sstream>

# include <boost/thread/recursive_mutex.hpp>

namespace TREX {
  namespace utils {

    /** @brief utils internals
     *
     * This namespace includes all classes and utilities used internally
     * for helping the utils implementation.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    namespace internals {
      class LogEntry;
    }
    
    /** @brief Thread safe text logging
     *
     * This class provide a thread safe mecanism to ensure that one
     * message produced by one thread won't be interrupted by another.
     * It provides an interface similar to C++ output streams.
     * It also allows to redirect all messages posted into syslog
     * with a "trex" name.
     *
     * The genral idea is that as long that one is cumulating << operators
     * this would represent one entry that cannot be cut. For example in the
     * following code :
     *
     * @code
     * TREX::utils::TextLog log("myflie.log");
     *
     * log<<"First text entry "<<'.'<<'.'<<std::endl<<" still first.";
     * log<<"Second entry"<<std::endl;
     * @endcode
     *
     * We ensure that the messages "First entry ..\n still first" and
     * "Second entry\n" won't be cut by another message. On the other hand
     * we allow other threads to wrtie a message between these two.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    class TextLog {
    public:
      /** @brief Default constructor.
       * Create a new instance without opening any output stream.
       */
      TextLog();
      /** @brief Constructor
       * @param file A symbolic file name
       * @param toSyslog Should the stream been duplicated into syslog ?
       *
       * Create a new instance connected to the file @e file. In
       * addition, provided that @e toSyslog is true all messages will
       * be duplicated into syslog.
       * @sa void open(std::string const &filem, bool toSyslog)
       */
      explicit TextLog(std::string const &file, bool toSyslog = false);
      /** @brief Destructor */
      ~TextLog();

      /** @brief output stream operator
       * @param val A value to display
       *
       * Starts a new entry to be written in the log staring with @e val .
       * @pre this instance is connected to a file. 
       *  
       * @return A LogEntry instance that will be used as a temporary
       * buffer to store the full entry before attempting to wrtie it into
       * the log as an atomic message.
       *
       * @sa class LogEntry
       * @sa void TextLog::write(std::string const &text);
       */
      template<typename Ty>
      internals::LogEntry operator<<(Ty const &val);

      /** @brief open log file
       *
       * @param file A symbolic file name
       * @param toSyslog Should the stream been duplicated into syslog ?
       *
       * Connect this entry to the log file @e name. In
       * addition, provided that @e toSyslog is true all messages will
       * be duplicated into syslog.
       */
      void open(std::string const &file, bool toSyslog = false);

    private:
      typedef boost::recursive_mutex mutex_type;
      typedef mutex_type::scoped_lock lock_type;

      mutex_type m_lock; //!< Entry writing protection muex
      std::ofstream m_log; //!< output stream
      bool m_syslog; //!< Syslog duplication flag

      /** @brief Atomic entry writing.
       *
       * @param text A text message
       *
       * This methods write the message @e text into the log in an atomic way.
       * Indeed it ensures that if a concurrent thread tries to write another
       * message at the same time @e text won't be interrupted. It is
       * internally used by @c internals::LogEntry to write the message it
       * buffered at its destruction
       *
       * @sa internals::LogEntry &TextLog::operator<<(Ty const &val)
       * @sa class internals::LogEntry
       */
      void write(std::string const &text);

      friend class internals::LogEntry;
    }; // TREX::utils::TextLog

    namespace internals {
      /** @brief Text Entry manager for TextLog
       *
       * This class is an utility created by TextLog to create a new atomic
       * log entry. It is a string stream that buffers all the message and
       * write it atomically to the TextLog instance that created him at
       * its destruction.
       * 
       * @relates TextLog
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup utils
       */
      class LogEntry {
      public:
	/** @brief Copy constructor.
	 *
	 * @param other The LogEntry to copy.
	 *
	 * To avoid duplicate entries this constructor clean the content
	 * of the buffer of @e other.
	 */
	LogEntry(LogEntry const &other);
	/** @brief Destructor
	 *
	 * Send the buffered message as an atomic entry to its creator.
	 */
	~LogEntry();

	/** @brief Stream operators
	 *
	 * These operators are classical interfaces for standard C++
	 * streams. They support any type supported by these streams
	 * including streams manipulators such as std::endl ...
	 * @{
	 */
	template<typename Ty>
	LogEntry &operator<<(Ty const &val);

	LogEntry &operator<<(std::ostream& (*f)(std::ostream&));
	/** @} */
      private:
	/** @brief Constructor
	 * 
	 * @param owner Creator
	 * Create a new instance connected to @e owner log
	 */
	explicit LogEntry(TextLog &owner);
    
	TextLog &m_owner; //!< destination log
	mutable std::auto_ptr<std::ostringstream> m_buff; //!< message buffer
    	
	// These function are not implemented in purpose
	LogEntry();
	void operator= (LogEntry const &other);

	friend class TREX::utils::TextLog;
      }; // TREX::utils::internals::LogEntry
    } // TREX::utils::internals

    // Templates

    template<typename Ty>
    internals::LogEntry TextLog::operator<<(Ty const &val) {
      internals::LogEntry entry(*this);
      return entry<<val;
    }

    namespace internals {
      /*
       * class LogEntry
       */ 
      template<typename Ty>
      LogEntry &LogEntry::operator<<(Ty const &val) {
	(*m_buff)<<val;
	return *this;
      }
  
      inline LogEntry &LogEntry::operator<<
        (std::ostream& (*f)(std::ostream&)) {
	(*m_buff)<<f;
	return *this;
      }

    }// TREX::utils::internals
  } // TREX::utils
} // TREX

#endif // _TEXTLOG_HH
