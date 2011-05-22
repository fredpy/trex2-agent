/* -*- C++ -*-
 * $Id$
 */
/** @file "LogManager.cc"
 * @brief LogManager implementation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>

#include<boost/tokenizer.hpp>

#include <cerrno>
#include <cstring>
#include <iomanip>

#include "LogManager.hh"
#include "TREXversion.hh"

using namespace TREX::utils; 

namespace {

  /** @brief Check for file exitsance
   * @param file A file name
   * @param mask Type of the file
   *
   * This method checks if @a file exists and if its type is consisistent
   * with @a mask. Mask is based on the @c stat  @c st_mode prodvided
   * attribute. The values can be a binary composition of the following
   * masks :
   * @li @c S_IFMT (default) any file type
   * @li @c S_IFIFO  named pipe (fifo) 
   * @li @c S_IFCHR  character special
   * @li @c S_IFDIR  directory
   * @li @c S_IFBLK  block special
   * @li @c S_IFREG  regular
   * @li @c S_IFLNK  symbolic link
   * @li @c S_IFSOCK socket
   * @li @c S_IFWHT  whiteout
   * For more details llook at the man stat(2)
   *
   * @retval true if @a file exists and is compaitible with @a mask
   * @retval false otherwise
   *
   * @author Frederic Py <fpy@mbari.org>
   * @relates class TREX::utils::LogManager
   */
  bool file_check(std::string const &file, mode_t mask = S_IFMT) {
    struct stat st;
    return 0==stat(file.c_str(), &st) && (mask&st.st_mode);
  }
  /** @brief short file name
   * @param file_name A sym,bolic file name
   *
   * This function gets rid of all the charcaters before the last @c '/'
   * of @a file_name. It is handy to get a short name or a file for
   * further manipulation.
   *
   * @return The remaining characters after the last @c '/' of @a file_name
   *
   * @author Frederic Py <fpy@mbari.org>
   * @relates class TREX::utils::LogManager
   */
  std::string short_name(std::string const &file_name) {
    size_t slash = file_name.find_last_of('/');
    
    if( std::string::npos!=slash )
      return std::string(file_name, slash+1);
    return file_name;
  }

}


/*
 * class LogManager
 */ 

// structors :

LogManager::LogManager():m_inited(false), m_level(TREX_LOG_LEVEL) {
}

LogManager::~LogManager() {
}

// methods :


std::string LogManager::locate(std::string const &file, bool &found) const {
  if( file.empty() )
    found = false;
  else {
    found = file_check(file);
    if( !found && '/'!=file[0] ) {
      std::list<std::string>::const_iterator i = m_searchPath.begin();
      for(; m_searchPath.end()!=i; ++i ) {
	std::string loc = *i+'/'+file;
	found = file_check(loc);
	if( found )
	  return loc;
      }
    }
  }
  return file;
}

bool LogManager::addSearchPath(std::string const &path) {
  if( !path.empty() ) {
    if( '.'==path[0] || file_check(path, S_IFDIR) ) {
      // This is a directory -> add it to the path
      m_searchPath.push_back(path);
      return true;
    }
  }
  return false;
}

void LogManager::loadSearchPath() {
  char *home = getenv(TREX_ENV);
  if( NULL!=home ) {
    std::string home_path(home);
    // Right now I am adding $TREX_HOME as it is
    // In  the future I may have to add $TREX_HOME/extra
    // -- or whatever I decide -- instead
    addSearchPath(home_path);
  }
  char *path = getenv(SEARCH_ENV);
  if( NULL!=path ) {
    // column separated list
    std::string path_str(path);
    static boost::char_delimiters_separator<char> sep(false, "", ":");
    boost::tokenizer<> tok(path_str, sep);
    for(boost::tokenizer<>::iterator beg=tok.begin(); beg!=tok.end();++beg) {
      addSearchPath(*beg);
    }
  }
}

std::string const &LogManager::logPath() {
  SharedVar<bool>::scoped_lock lock(m_inited);

  if( !*m_inited ) {
    if( m_path.empty() ) {
      createLatest();
    } else if( !file_check(m_path) ) { 
      int ret = mkdir(m_path.c_str(), 0777);
      if( 0==ret ) {
	throw ErrnoExcept("LogManager: Unable to create log directory :");
      }
    }
    std::string cfg = m_path+"/cfg";
    int ret = mkdir(cfg.c_str(), 0777);
    if( 0!=ret && EEXIST!=errno )
      throw ErrnoExcept("LogManager: error while creating cfg dir");
    m_syslog.open(m_path+'/'+TREX_LOG_FILE);
    m_syslog<<"TREX version "<<TREX::version::str();
    loadSearchPath();
    *m_inited = true;
  }
  return m_path;
}

void LogManager::createLatest() {
  char *base_dir = getenv(LOG_DIR_ENV);
  char dated_dir[17];
  
  if( NULL==base_dir ) {
    throw ErrnoExcept("LogManager: $"LOG_DIR_ENV" is not set");
  }
  dated_dir[16] = '\0';

  time_t cur_time;
  time(&cur_time);
  strftime(dated_dir, 16, "/%Y.%j.", gmtime(&cur_time));
  
  std::string latest = base_dir;
  std::string cur_basis = base_dir;
  size_t len = latest.length()+30;
  char *buf = new char[len+1];
  std::memset(buf, 0, len+1);
  latest = latest+'/'+LATEST_DIR;
  cur_basis += dated_dir;
  int ret = readlink(latest.c_str(), buf, len-1);
  size_t last = 0;
  if( ret>0 ) {
    // latest link exists and point to something
    size_t b_len = cur_basis.length();
    if( 0==cur_basis.compare(0, b_len, buf, b_len) ) {
      std::istringstream iss(std::string(buf, b_len));
      iss>>last;
    }
    unlink(latest.c_str());
  }
  delete[] buf;
  for(unsigned i=1; i<MAX_LOG_ATTEMPT; ++i) {
    std::ostringstream oss;
    oss<<cur_basis<<std::setw(2)<<std::setfill('0')<<(last+i);
    int ret = mkdir(oss.str().c_str(), 0777);
    if( 0==ret ) {
      m_path = oss.str();
      symlink(m_path.c_str(), latest.c_str());
      return;
    } else if( EEXIST!=errno )
      throw ErrnoExcept("LogManager: error while trying to create log dir:");
  }
  throw ErrnoExcept("LogManager", "Too many attempts ... clean up your "
		    "log directory");
}

std::string LogManager::file_name(std::string const &short_name) {
  return logPath()+'/'+short_name;
}

std::string LogManager::getCfgPath() {
  return file_name("cfg/");
}

std::string LogManager::use(std::string const &file_name, bool &found) {
  std::string dest_name = getCfgPath()+short_name(file_name);

  std::string located = locate(file_name, found);
  if( found ) {
    std::ifstream src(located.c_str(), std::ios::binary);
  
    if( !!src ) {
      syslog()<<"- Using file "<<located;
      std::ofstream dest(dest_name.c_str(), std::ios::binary);

      dest<<src.rdbuf();
      src.close();
      dest.close();
    } else
      throw ErrnoExcept("File \""+located+'\"');
  }
  return located;
}

TextLog &LogManager::syslog() {
  return m_syslog;
}

internals::LogEntry LogManager::syslog(std::string const &txt) {
  return m_syslog<<('['+txt+"] ");
}

bool LogManager::setLogLevel(LogLevel lvl) {
  SharedVar<bool>::scoped_lock guard(m_inited);
  if( !*m_inited ) {
    m_level = lvl;
    return true;
  }
  syslog("LogManager")<<"Cannot change verbosity level after init."<<std::endl;
  return false;
}

bool LogManager::setLogPath(std::string const &path) {
  SharedVar<bool>::scoped_lock guard(m_inited);
  if( !*m_inited ) {
    m_path = path;
    return true;
  }
  syslog("LogManager")<<"Cannot change log path after init."<<std::endl;
  return false;
}


