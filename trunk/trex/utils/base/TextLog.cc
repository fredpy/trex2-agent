/* -*- C++ -*- */
/** @file "TextLog.cc"
 * @brief TextLog implementation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */
#include <syslog.h>

#include "TextLog.hh"

/** @brief syslog identifier
 *
 * This identifier is usd when the log is sent to syslog. It is
 * the name given as the source of the message
 *
 * @relates TREX::utils::TextLog
 * @ingroup utils
 */
#define TREX_SYSLOG_NAME "trex"

using namespace TREX::utils;
/*
 * class TextLog
 */ 

// structors :

TextLog::TextLog():m_syslog(false) {}

TextLog::TextLog(std::string const &name, bool toSyslog)
  :m_log(name.c_str()), m_syslog(toSyslog) {
  if( m_syslog ) {
    ::openlog(TREX_SYSLOG_NAME, LOG_CONS, 0);
  }
}

TextLog::~TextLog() {
  if( m_syslog )
    ::closelog();
}

// Manipulators :

void TextLog::open(std::string const &name, bool toSyslog) {
  lock_type locker(m_lock);
  
  m_log.open(name.c_str());
  if( toSyslog!=m_syslog ) {
    m_syslog = toSyslog;
    if( m_syslog ) 
      ::openlog(TREX_SYSLOG_NAME, LOG_CONS, 0);
    else
      ::closelog();
  }
}

void TextLog::write(std::string const &text) {
  lock_type locker(m_lock);
  char const *str = text.c_str();
  size_t len = text.length();

  m_log.write(str, len);
  if( m_syslog ) 
    ::syslog(LOG_ERR, "%s", str);
  else
    m_log.flush();
//   m_log<<text<<std::flush;
}

/* 
 * class LogEntry 
 */


// structors :

internals::LogEntry::LogEntry(TREX::utils::TextLog &owner)
  :m_owner(owner), m_buff(new std::ostringstream) {
}

internals::LogEntry::LogEntry(internals::LogEntry const &other) 
  :m_owner(other.m_owner), m_buff(other.m_buff) {
}

internals::LogEntry::~LogEntry() {
  if( 0!=m_buff.get() ) {
    std::string to_log = m_buff->str();
    if( !to_log.empty() ) {
      if( '\n'!=to_log[to_log.length()-1] )
	to_log.push_back('\n');
      m_owner.write(to_log);
    }
  }
}
