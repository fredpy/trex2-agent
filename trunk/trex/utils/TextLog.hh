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
#ifndef H_trex_utils_TextLog
# define H_trex_utils_TextLog

# include "Symbol.hh"
# include "SharedVar.hh"

# include <memory>
# include <set>
# include <list>

# include <boost/thread/thread.hpp>
# include <boost/thread/recursive_mutex.hpp>
# include <boost/thread/condition_variable.hpp>
# include <boost/iostreams/stream.hpp>
# include <boost/optional.hpp>
# include <boost/tuple/tuple.hpp>

namespace TREX {
  namespace utils {

    class TextLog;

    namespace internals {

      /** @brief log entry
       * 
       * This class is used as the basic container of a log entry 
       * as it is produced through an entry_sink. It maintains 
       * the TextLog that created it along with an optional creation 
       * date, a source and a kind along with the message as it is is 
       * build.
       *
       * When it is destroyed it will send all the informationto the 
       * TextLog that can then consider it as a complete entry to be 
       * sent to its listeners
       *
       * @relates TextLog
       * @ingroup transaction
       * @author Frederic Py
       */
      class entry {
      public:
	/** @brief identifiers type
	 * 
	 * The type used for both the source and kind of an entry
	 */
        typedef Symbol                id_type;
	/** @brief message type
	 *
	 * The type used to store the message content of en entry
	 */
        typedef std::string           msg_type;
	/** @brief date type
	 *
	 * The type used for a data when available.
	 *
	 * @note for convenience we used a type which correspoinf to  
	 * a TREX TICK. A more generic implementation would probabaly 
	 * use a more general type to represent a real date
	 */
        typedef unsigned long long    date_type;
        typedef msg_type::value_type  char_type;
        typedef msg_type::size_type   size_type;

	/** @brief destructor 
	 *
	 * If the message content is not empty. This method will 
	 * notify the TextLog of the message. 
	 *
	 * @sa TextLog::send
	 */
        ~entry();
	/** @brief append new content
	 *
	 * @param s A pointer to the characters to append
	 * @param n The number of character to append from @p s
	 *
	 * This method is used by entry_sink in order to append new content 
	 * to the message of this entry
	 *
	 * @return @p n 
	 */
        size_type write(char_type const *s, size_type n) {
          m_msg.append(s, n);
          return n;
	}
        
      private:
	/** @brief Creator 
	 *
	 * The TextLog that created this entry
	 */
        TextLog &m_dest;
	/** @brief date field
	 *
	 * The optional date of creation
	 */
        boost::optional<date_type> m_date;
	/** @brief source field
	 *
	 * The source of the entry
	 */
        Symbol m_source;
	/** @brief message kind
	 *
	 * The kind of the entry
	 */
        Symbol m_kind;
	/** @brief message content
	 *
	 * The mesage content of this entry
	 */
        std::string m_msg;

	/** @brief Constructor 
	 *
	 * @param dest A destination TextLog
	 * @param src A source identifier
	 * @param kind A message kind identifier
	 *
	 * Create a new entry of the type @p kind, to be sent to @p dest 
	 * with no date and @p src as a source.
	 */
        entry(TextLog &dest, Symbol const &src, Symbol const &kind)
        :m_dest(dest), m_source(src), m_kind(kind) {}
	/** @brief Constructor 
	 *
	 * @param dest A destination TextLog
	 * @param date A date 
	 * @param src A source identifier
	 * @param kind A message kind identifier
	 *
	 * Create a new entry of the type @p kind, to be sent to @p dest,
	 * created on @p date and with @p src as a source.
	 */
        entry(TextLog &dest, date_type const &date,
            Symbol const &src, Symbol const &kind)
        :m_dest(dest), m_date(date), m_source(src), m_kind(kind) {}

        friend class TREX::utils::TextLog;
      }; 

      class LogEntry;

      /** @brief entry streming utility 
       *
       * A class used as a proxy to build the message of an entry using 
       * a standard output stream.
       *
       * @relates LogEntry
       * @ingroup transaction
       * @author Frederic Py
       * @sa class entry
       */
      class entry_sink {
      public:
        typedef entry::char_type           char_type;
        typedef boost::iostreams::sink_tag category;

	/** @brief Copy constructor 
	 *
	 * @param[in] other Another instance 
	 *
	 * Create a new instance and transfert tjhe ownership of the 
	 * entry formerly associated to @p other to this new instance.
	 *
	 * This tranfert of ownership ensure that there will be only one 
	 * stream writing to a given @p entry.
	 *
	 * @post other is not associated to an entry anymore
	 */
        entry_sink(entry_sink const &other)
        :m_entry(other.m_entry) {}
	/** @brief destructor
	 *
	 * If the entry is ossaciated to an entry it will destroy it, 
	 * resulting on a new message send ot the entry TextLog
	 *
	 * @sa entry::~entry()
	 */
        ~entry_sink() {
          m_entry.reset();
        }

	/** @brief write operation
	 *
	 * @param s A pointer to the characters to append
	 * @param n The number of character to append from @p s
	 *
	 * If this instance is associated to en entry it will write the 
	 * @p n characters pointed by @p s to this entry. Otherwise it will 
	 * do nothing.
	 *
	 * @retval @p n if the insatnce is associated to en entry
	 * @retval 0 otherwise
	 */
        std::streamsize write(char_type const *s, std::streamsize n) {
          if( NULL!=m_entry.get() )
            return m_entry->write(s, n);
          return 0;
        }

      private:
        mutable std::auto_ptr<entry> m_entry;

	/** @brief Constructor
	 *
	 * @param e A pointer to an entry
	 *
	 * Create a new instance  associated to the entry pointed by @p e
	 */
        explicit entry_sink(entry *e):m_entry(e) {}

        friend class LogEntry;
      };


      /** @brief A temporary local stream
       *
       * This class implements and maintain a stream to build a new entry
       * to be sent to a TextLog. As the only public constructor of this 
       * class is a "copy" constructor that transfert ownershipe of the 
       * stream to the newly created entry it is a temproaray stream that 
       * will release a new entry as soon as its last instance is destroyed.
       *
       * It is used as the core mechanism to produces new entries which are 
       * atomically sent to the TextLog while apprently using a typical 
       * stream.
       *
       * @relates TextLog
       * @ingroup transaction
       * @author Frederic Py 
       */
      class LogEntry {
      public:
	/** @brief subjacent stream type
	 *
	 * The standard output stream used to implement this class.
	 * This stream is based on entry_sink 
	 *
	 * @sa entry_sink
	 */
        typedef boost::iostreams::stream<entry_sink> stream_type;

	/** @brief Copy constructor
	 *
	 * @param[in] other another instance 
	 *
	 * Create a new instance that takes control of the entry (if any)
	 * @p other was writing to.
	 *
	 * @post @p other is no more associated to en entry
	 */
        LogEntry(LogEntry const &other) :m_stream(*other.m_stream) {}

	/** @brief Output stream operator
	 *
	 * @tparam Ty A type
	 * @param x An insatnce of @p Ty
	 * 
	 * Call the operator @c << of the subjacent stream of this 
	 * entry with @p x as an argument. 
	 *
	 * This aloow to make this instance appear like a regular 
	 * stream to the caller.
	 *
	 * @return the subjacent stream of this instance 
	 */
        template<typename Ty>
        std::ostream &operator<<(Ty const &x) {
          return m_stream<<x;
        }

	/** @brief subjacent stream
	 *
	 * @return he stream maintained by this instance
	 */
	std::ostream &stream() const {
	  return m_stream;
	}
	/** @brief implicit ostream ocnversion
	 *
	 * This operator allow to convert a LogEntry to a 
	 * standard output stream.
	 */
	operator std::ostream &() const {
	  return stream();
	}

      private:
	/** @brief Constructor 
	 *
	 * @param ref An entry
	 *
	 * Create a new instance associated to the entry pointed by 
	 * @p ref.
	 */
        LogEntry(entry *ref)
        :m_stream(entry_sink(ref)) {}

        mutable stream_type m_stream;

        friend class TREX::utils::TextLog;

        // purposely no code
        LogEntry();
      };
    } // TREX::utils::internals

    /** @brief Empty symbol
     *
     * An  empty symbol used for entries with no type
     *
     * @relates TextLog
     */
    extern Symbol const null;
    /** @brief info symbol
     *
     * A symbol used as the kind for info entries
     *
     * @relates TextLog
     */
    extern Symbol const info;
    /** @brief Warning symbol
     *
     * A symbol used as the kind for warning entries 
     *
     * @relates TextLog
     */
    extern Symbol const warn;
    /** @brief Error symbol
     *
     * A symbol used as the kind for error entries 
     *
     * @relates TextLog
     */
    extern Symbol const error;

    /** @brief Logging text entries management class
     *
     * This class implements the core mechanism for trex to log
     * text messages that are atomic, relativeley thread safe and 
     * can be dispatched to multiple listeners. All the messages (or 
     * entry) have an associated source, kind and optianally a date
     *
     * The entries are built by the programmer using standard C++ 
     * streams that when released will produce a new entry that this 
     * class will then send to a primary listener and eventually dispatch 
     * to exsting secondary listeners that are ran in another thread.
     *
     * All the syslog() methods found accros TREX classes are supported 
     * by this class.
     *
     * @ingroup transaction
     * @euthor Frederic Py
     */
    class TextLog {
    public:
      typedef internals::entry::id_type   id_type;
      typedef internals::entry::msg_type  msg_type;
      typedef internals::entry::date_type date_type;
      typedef internals::LogEntry         stream_type;

      /** @brief Default constructor 
       *
       * Create a new instance with no primary handler
       */
      TextLog() {}
      /** @brief Constructor
       *
       * @tparam Handler a handler type
       * @param[in] primary An instance of @p Handler
       *
       * Create a new instance that will use a copy of @p primary
       * as its primary handler.
       *
       * @pre Handler is copy contructible
       * @pre Handler derives from handler
       *
       * @note It is hgihly recommended to have Handler class to be thread 
       * safe with is message method protected against reentrance. Indeed 
       * the primary handler of a TextLog can be called an any threads and 
       * concurrently.
       */
      template<class Handler>
      TextLog(Handler const &primary)
      :m_primary(new Handler(primary)) {}
      /** @brief Destructor 
       * 
       * destroy this instance and all the handlers associated to it.
       */
      ~TextLog();

      /** @brief set primary handler
       *
       * @tparam Handler a handler type
       * @param[in] primary An instance of @p Handler
       *
       * If this instance does not yet have a primary handler, this call 
       * make a copy of @p primary as its primary handler.
       *
       * @pre Handler is copy contructible
       * @pre Handler derives from handler
       *
       * @retval true if this call resulted on a copy of @p handler becoming
       * this instance primary handler.
       * @retval false if this instance had already a rpimary handler
       *
       * @note It is highly recommended to have Handler class to be thread 
       * safe with is message method protected against reentrance. Indeed 
       * the primary handler of a TextLog can be called an any threads and 
       * concurrently.
       */
      template<class Handler>
      bool set_primary(Handler const &primary) {
        if( NULL==m_primary.get() ) {
          m_primary.reset(new Handler(primary));
          return true;
        }
        return false;
      }

      /** @brief Ad secondary handler
       *
       * @tparam Handler a handler type
       * @param handle An instance of Handler
       *
       * Add a copy of @p handle as a secondary handler of this instance
       *
       * @pre Handler is copy contructible
       * @pre Handler derives from handler
       *
       * @post A copy of @p handle is now associated to this TextLog that
       * will notify it of new entries within a specific thread created by 
       * this instance for secondary handlers.
       *
       * @note As all the call of message for secondary handler are made in 
       * a specific thread this call does not need to be protected against 
       * reentrance for @p Handler. Do note also that as opposed to the primary 
       * handler the notification of nrew entries may be delayed.
       */
      template<class Handler>
      void add_handler(Handler const &handle) {
        std::auto_ptr<handler> h(new Handler(handle));
        bool first = false;
        h->m_log = this;
        {
          scoped_lock guard(m_lock);
          first = !m_running;
          m_running = true; // ensure that no one will try to create a thread
          m_handlers.insert(h.release());
        }
        if( first ) {
          if( NULL!=m_thread.get() )
            m_thread->join();
          m_thread.reset(new boost::thread(thread_proxy(this)));
        }
      }

      /** @brief new entry
       * 
       * @param[in] date date of the entry
       * @param[in] from source of the entry
       * @param[in] kind type of the entry
       *
       * Create a new empty entry that can be fileld using standard output 
       * stream operations
       *
       * @return A temporary stream to build this entry
       * @{
       */
      stream_type msg(date_type const &when, 
          id_type const &from, id_type const &kind=null) {
        return stream_type(new internals::entry(*this, when, from, kind));
      }
      stream_type operator()(long long when,
          id_type const &from,
          id_type const &kind=null) {
        return msg(when, from, kind);
      }
      stream_type msg(id_type const &from, id_type const &kind=null) {
        return stream_type(new internals::entry(*this, from, kind));
      }
      stream_type operator()(id_type const &from, 
          id_type const &kind=null) {
        return msg(from, kind);
      }
      /** @} */

      /** @brief Log entry handler
       *
       * The base class used to implement any TextLog entry handler. 
       *
       * A handler is just a class that will be notified of new entries 
       * created and can then process them. Ty[pical processing would be 
       * to redirect them into a stream
       *
       * @relates TextLog
       * @ingroup transaction
       * @author Frederic Py 
       */
      class handler {
      public:
        typedef TextLog::id_type   id_type;
        typedef TextLog::msg_type  msg_type;
        typedef TextLog::date_type date_type;

	/** @brief default constructor */
        handler()
        :m_log(NULL) {}
	/** @brief Copy constructor */
        handler(handler const &other)
        :m_log(NULL) {}
	/** @brief Destructor 
	 *
	 * If the handler was associated to a TextLog, 
	 * notifies the TextLog about its destruction
	 */
        virtual ~handler();

      protected:
	/** @brief Check if associated to a TextLog
	 *
	 * @retval true if associated to a TextLog
	 * @retval false otherwise
	 */
        bool is_attached() {
          return NULL!=m_log;
        }
	/** @brief Detach from TextLog
	 *
	 * This method allow to detach the handler from its associated 
	 * TextLog when possible.
	 * @note As of now we do not allow to detach a primary handler 
	 * from it
	 * @retval true if succesfully detached
	 * @retval false otherwise
	 */
        bool detach();

	/** @brief New entry
	 *
	 * This callback notifeds attached handlers from a new message
	 */
        virtual void message(boost::optional<date_type> const &date,
            id_type const &who, id_type const &kind,
            msg_type const &what) =0;

      private:	
        bool detach(bool);
        TextLog *m_log;

        friend class TextLog;
      }; // TREX::utils::TextLog::handler

    private:
      typedef boost::tuple< boost::optional<date_type>,
          id_type, id_type,
          msg_type > packet;
      typedef SharedVar< std::list<packet> > queue_type;
      typedef std::set<handler *> handler_set;

      class thread_proxy {
      public:
        thread_proxy(thread_proxy const &other)
        :m_log(other.m_log) {}
        ~thread_proxy() {}

        void operator()();

      private:
        bool next(TextLog::packet &msg);

        thread_proxy(TextLog *me)
        :m_log(me) {}

        TextLog *m_log;
        friend class TextLog;
      };
      friend class thread_proxy;

      typedef boost::recursive_mutex  mutex_type;
      typedef mutex_type::scoped_lock scoped_lock;

      void send(boost::optional<date_type> const &when,
          id_type const &who, id_type const &type,
          msg_type const &what);

      std::auto_ptr<handler> m_primary;

      boost::condition_variable_any m_have_message;
      queue_type m_queue;

      bool is_running() const {
        scoped_lock guard(m_lock);
        return m_running;
      }
      bool stop() {
        bool ret = false;
        {
          scoped_lock guard(m_lock);
          std::swap(m_running, ret);
        }
	m_have_message.notify_one();
        return ret;
      }

      std::auto_ptr<boost::thread> m_thread;

      mutable mutex_type m_lock;
      bool       m_running;
      handler_set m_handlers; 

      friend class handler;
      friend class internals::entry;      
    }; // TREX::utils::TextLog

    namespace internals {

      /** @brief Optional mutex
       *
       * @tparam Reentrant mutex enabling flag
       *
       * This template class is an helper to make the thread safety of code being 
       * parametrized during compilation. Indeed depending on the value of @p Reentrant
       * this class can be either a true mutex or a fake one doing nothing when requested 
       * to be locked or unlocked. Its interface is compatible with boost mutex 
       * implementation.
       * 
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup utils
       */
      template<bool Reentrant>
      struct optional_mtx {
        /** @brief Mutex lock
         *
         * This method increased the locking level of the current thread. 
         * In the case that the mutex is not currently locked by the calling 
         * thread, the call will block untils this thread can take ownership 
         * of this mutex  
         *
         * @post the mutex is locked
         *
         * @note If @p Reentrant is @c false this call will do nothing 
         */
        void lock() {}
        /** @brief Mutex release
         *
         * Release the blocked mutex. This decrease the level of ownership of 
         * this mutex by the calling thread. If the level reaches 0 then the 
         * thread is set free for other threads to lock it
         *
         * @note If @p Reentrant is @c false this call will do nothing         
         */
        void unlock() {}
      }; // TREX::utils::details::optional_mtx<>

#ifndef DOXYGEN      
      template<>
      struct optional_mtx<true> {
        void lock() {
          m_mtx.lock();
        }
        void unlock() {
          m_mtx.unlock();
        }

      private:
        boost::recursive_mutex m_mtx;
      }; // TREX::utils::details::optional_mtx<true>
#endif // DOXYGEN

    } // TREX::

    /** @brief log file messsage handler
     *
     * @tparam Reentrant thread safety flag
     *
     * This template class implements a log message handler that redirect 
     * all the messages produced into an output log file.
     *
     * @p Reentrant is  boolean indicating whther the handler need to be 
     * thread safe or not. The non reentrant instance of this template allows 
     * for faster execution (as its mutex is a dummy) at the risk of unexpected 
     * behavior if executed by multiple thread instances.  
     *
     * @author Frederic Py <fpy@mbari.org>
     * ingroup utils
     */
    template<bool Reentrant>
    class basic_log_file :public TextLog::handler {
      /** @brief mutex type
       *
       * The type of the mutex for tread safety. If @p Reentrant is false
       * this would be a dummy mutex doing nothing.
       */
      typedef internals::optional_mtx<Reentrant> mutex_type;
      typedef boost::lock_guard<mutex_type>      lock_type;
    public:
      basic_log_file() {}
      basic_log_file(std::string const &file)
      :m_file(new std::ofstream(file.c_str())) {}
      basic_log_file(basic_log_file const &other)
      :m_file(other.m_file) {}
      ~basic_log_file() {}

    private:
      void message(boost::optional<date_type> const &date,
          id_type const &who, id_type const &kind,
          msg_type const &what) {
        if( NULL!=m_file.get() ) {
          std::ostringstream oss;
          if( date )
            oss<<'['<<(*date)<<']';
          if( !who.empty() )
            oss<<'['<<who<<']';
          if( null!=kind && info!=kind )
            oss<<kind<<": ";
          else if( !oss.str().empty() )
            oss.put(' ');
          oss<<what;
          {
            lock_type guard(m_mtx);
            m_file->write(oss.str().c_str(), oss.str().size());
            m_file->flush();
          }
        }
      }

      mutable std::auto_ptr<std::ofstream> m_file;
      mutex_type m_mtx;
    }; // TREX::utils::log_file    

    /** @brief Default `basic_log_file` specialization
     *
     * A specialization of `basic_log_file` set up to not being reentrant. 
     * 
     * @ingroup utils
     * @relates basic_log_files
     */
    typedef basic_log_file<false> log_file;

  } // TREX::utils
} // TREX

#endif // H_trex_utils_TextLog
