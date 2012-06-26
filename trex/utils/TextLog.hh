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

# include <memory>
# include <set>

# include <boost/thread/recursive_mutex.hpp>
# include <boost/iostreams/stream.hpp>

namespace TREX {
  namespace utils {

    class TextLog;

    namespace internals {
      
      class entry {
      public:
	typedef Symbol                id_type;
	typedef std::string           msg_type;
	typedef msg_type::value_type  char_type;
	typedef msg_type::size_type   size_type;
	
	~entry();
	size_type write(char_type const *s, size_type n) {
	  m_msg.append(s, n);
	  return n;
	}
      private:
	TextLog &m_dest;
	Symbol m_source;
	Symbol m_kind;
	std::string m_msg;
	
	entry(TextLog &dest, Symbol const &src, Symbol const &kind) 
	  :m_dest(dest), m_source(src), m_kind(kind) {}

	friend class TextLog;
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

	friend class TextLog;
	
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
      typedef internals::entry::id_type  id_type;
      typedef internals::entry::msg_type msg_type;
      typedef internals::LogEntry        stream_type;
  
      TextLog() {}
      ~TextLog();

      stream_type msg(id_type const &from, id_type const &kind=null) {
	return stream_type(new internals::entry(*this, from, kind));
      }
      stream_type operator()(id_type const &from, 
			     id_type const &kind=null) {
	return msg(from, kind);
      }

      class handler {
      public:
	typedef TextLog::id_type  id_type;
	typedef TextLog::msg_type msg_type;

	handler()
	  :m_log(NULL) {}
	virtual ~handler() {
	  detach();
	}

	bool is_attached() const {
	  return NULL!=m_log;
	}
	void detach();

      protected:
	virtual void message(id_type const &who, id_type const &kind, 
			     msg_type const &what) =0;

      private:	
	TextLog *m_log;
	
	friend class TextLog;
      }; // TREX::utils::TextLog::handler

      bool add_handler(handler &handle);
      bool remove_handler(handler &handle) {
	if( this==handle.m_log ) {
	  handle.detach();
	  return true;
	}
	return false;
      }	  
	
    private:
      typedef boost::recursive_mutex  mutex_type;
      typedef mutex_type::scoped_lock scoped_lock;

      void send(id_type const &who, id_type const &type, 
		msg_type const &what);

      mutex_type m_lock;
      std::set<handler *> m_handlers; 

      friend class handler;
      friend class internals::entry;      
    }; // TREX::utils::TextLog

    class log_file :public TextLog::handler {
    public:
      log_file() {}
      log_file(std::string const &file)
	:m_file(file.c_str()) {} 
      ~log_file() {}

      void open(std::string const &file) {
	m_file.open(file.c_str());
      }

    private:
      void message(id_type const &who, id_type const &kind, 
		   msg_type const &what) {
	if( null==kind || info==kind )  {
	  if( !who.empty() )
	    m_file<<'['<<who<<"] ";
 	} else {
	  if( !who.empty() )
	    m_file<<'['<<who<<']';
	  m_file<<kind<<": ";
	}       
	m_file<<what<<std::flush;
      }
      std::ofstream m_file;
    }; // TREX::utils::log_file    
    

  } // TREX::utils
} // TREX

#endif // H_trex_utils_TextLog
