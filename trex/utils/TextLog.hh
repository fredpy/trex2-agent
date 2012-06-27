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

      class entry {
      public:
        typedef Symbol                id_type;
        typedef std::string           msg_type;
        typedef unsigned long long    date_type;
        typedef msg_type::value_type  char_type;
        typedef msg_type::size_type   size_type;

        ~entry();
        size_type write(char_type const *s, size_type n) {
          m_msg.append(s, n);
          return n;
	}
        
      private:
        TextLog &m_dest;
        boost::optional<date_type> m_date;
        Symbol m_source;
        Symbol m_kind;
        std::string m_msg;

        entry(TextLog &dest, Symbol const &src, Symbol const &kind)
        :m_dest(dest), m_source(src), m_kind(kind) {}
        entry(TextLog &dest, date_type const &date,
            Symbol const &src, Symbol const &kind)
        :m_dest(dest), m_date(date), m_source(src), m_kind(kind) {}

        friend class TREX::utils::TextLog;
      }; 

      class log_entry;

      class entry_sink {
      public:
        typedef entry::char_type           char_type;
        typedef boost::iostreams::sink_tag category;

        entry_sink(entry_sink const &other)
        :m_entry(other.m_entry) {}
        ~entry_sink() {
          m_entry.reset();
        }

        std::streamsize write(char_type const *s, std::streamsize n) {
          if( NULL!=m_entry.get() )
            return m_entry->write(s, n);
          return 0;
        }

      private:
        mutable std::auto_ptr<entry> m_entry;

        explicit entry_sink(entry *e):m_entry(e) {}

        friend class LogEntry;
      };


      class LogEntry {
      public:
        typedef boost::iostreams::stream<entry_sink> stream_type;

        LogEntry(LogEntry const &other) :m_stream(*other.m_stream) {}

        template<typename Ty>
        std::ostream &operator<<(Ty const &x) {
          return m_stream<<x;
        }

      private:
        LogEntry(entry *ref)
        :m_stream(entry_sink(ref)) {}

        mutable stream_type m_stream;

        friend class TREX::utils::TextLog;

        // purposely no code
        LogEntry();
      };
    } // TREX::utils::internals

    extern Symbol const null;
    extern Symbol const info;
    extern Symbol const warn;
    extern Symbol const error;

    class TextLog {
    public:
      typedef internals::entry::id_type   id_type;
      typedef internals::entry::msg_type  msg_type;
      typedef internals::entry::date_type date_type;
      typedef internals::LogEntry         stream_type;

      TextLog() {}
      template<class Handler>
      TextLog(Handler const &primary)
      :m_primary(new Handler(primary)) {}
      ~TextLog();

      template<class Handler>
      bool set_primary(Handler const &primary) {
        if( NULL==m_primary.get() ) {
          m_primary.reset(new Handler(primary));
          return true;
        }
        return false;
      }

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

      stream_type msg(id_type const &from, id_type const &kind=null) {
        return stream_type(new internals::entry(*this, from, kind));
      }
      stream_type msg(date_type const &when, 
          id_type const &from, id_type const &kind=null) {
        return stream_type(new internals::entry(*this, when, from, kind));
      }
      stream_type operator()(id_type const &from, 
          id_type const &kind=null) {
        return msg(from, kind);
      }
      stream_type operator()(long long when,
          id_type const &from,
          id_type const &kind=null) {
        return msg(when, from, kind);
      }

      class handler {
      public:
        typedef TextLog::id_type   id_type;
        typedef TextLog::msg_type  msg_type;
        typedef TextLog::date_type date_type;

        handler()
        :m_log(NULL) {}
        handler(handler const &other)
        :m_log(NULL) {}
        virtual ~handler() {
          detach();
        }

      protected:
        bool is_attached() {
          return NULL!=m_log;
        }
        bool detach();

        virtual void message(boost::optional<date_type> const &date,
            id_type const &who, id_type const &kind,
            msg_type const &what) =0;

      private:	
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

      template<bool Reentrant>
      struct optional_mtx {
        void lock() {}
        void unlock() {}
      };

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
      };

    } // TREX::

    template<bool Reentrant>
    class basic_log_file :public TextLog::handler {
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

    typedef basic_log_file<false> log_file;

  } // TREX::utils
} // TREX

#endif // H_trex_utils_TextLog
