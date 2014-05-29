/** @file "trex/utils/LogManager.hh"
 * @brief LogManager class definition
 * 
 * This header defines the LogManager utility alogn with several macro and 
 * functiona that eases file management in TREX
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
#ifndef H_LogManager
# define H_LogManager

# include <list>
# include <boost/filesystem.hpp>

# include "platform/memory.hh"
# include "platform/system_error.hh"

# include "log/log_pipe.hh"
# include "log/out_file.hh"

# include "singleton.hh"


/** @brief TREX default log file name
 *
 * This macro is used by LogManager. It gives the name of the log
 * file to create for logging text messages.
 *
 * @relates TREX::utils::LogManager
 * @author Frederic Py <fpy@mbari.org>
 */
# define TREX_LOG_FILE "TREX.log"

/** @brief Home environment variable
 * 
 * The name of the environment variable that indicates where TREX is located. 
 * If valid, the value of this variable will be automatically added to the 
 * search path used by LogManager to @c locate new files 
 *
 *
 * @relates TREX::utils::LogManager
 * @author Frederic Py <fpy@mbari.org>
 * @sa LogManager::m_searchPath
 */
# define TREX_ENV    "TREX_HOME"

/** @brief Search path environment variable
 * 
 * The name of the environment variable that indicates where TREX can search for
 * files to locate them. The format fro this variable is a list of directories 
 * separated by a column (':')
 *
 * @relates TREX::utils::LogManager
 * @author Frederic Py <fpy@mbari.org>
 * @sa LogManager::m_searchPath
 */
# define SEARCH_ENV  "TREX_PATH"

/** @brief log directory environment variable
 *
 * This macro gives the name of the environment variable to read
 * for knowing in what path the log files should be placed. 
 *
 * @relates TREX::utils::LogManager
 * @author Frederic Py <fpy@mbari.org>
 */
# define LOG_DIR_ENV "TREX_LOG_DIR"
/** @brief Name of the symbolic link to "latest" logs
 *
 * This macro gives the name to give to the symbolic link that
 * will be created in the log directory to point to the logs of
 * the latest/current session
 *
 * @relates TREX::utils::LogManager
 * @author Frederic Py <fpy@mbari.org>
 */
# define LATEST_DIR "latest"
/** @brief Maximum log directory creation attempts
 *
 * This macro indicates how much attempt to create a new log directory
 * should be done before giving up and generating an error.
 * 
 * @note This value @b do @b not mean that TREX can only execute up to 
 *       @c MAX_LOG_ATTEMPT missions in a single day. Indeed if one look
 *       at the code of LogManager::createLatest he will see that it makes this 
 *       number of attempt stariung from the directory referred by @c LATEST_DIR
 *       link. Such behavior should really happen if the symbolic link 
 *       @c LATEST_DIR is not set properly. 
 *
 * @relates TREX::utils::LogManager
 * @author Frederic Py <fpy@mbari.org>
 */
# define MAX_LOG_ATTEMPT 1024

namespace TREX {
  namespace utils {

    namespace details {
      /** @brief log_manager implementation details
       *
       * This class impelement most of the main functionalities of 
       * log_manager class. Its interface and implementation are 
       * hidden from the rest of the API so any modification at 
       * implementation level has a  limited impact on other components 
       * using log_manager
       *
       * @relates TREX::utils::log_manager
       * @author Frederic Py <fredpy@gmail.com>
       */
      class mgmt_impl;
    } // TREX::utils::details
    
    
    /** @brief Logging and file access management for trex
     *
     * This class is a gloabl utility for trex components in order 
     * to log information and access files. It provides ways to produce 
     * log messages with atomic write (avoiding messages intertwined due
     * to multithread access); access to any log messsages as they are 
     * provided; management and stroage of all the log files within a 
     * single directory; localisation of files using TREX_PATH 
     * environement variable. 
     *
     * In addition this class manages the shared asio threads used 
     * within trex. It creates an io service which is executed within 
     * a thread pool, clients of this class can uses this class to have 
     * access to this ioservice and manage the number of threads that 
     * are executing this service
     *
     * @note This class can be accessed only as a singleton
     *
     * @sa TREX::utils::singleton::use
     *
     * @ingroup utils
     * @author Frederic Py <fredpy@gmail.com>
     */
    class log_manager :boost::noncopyable {
    public:
      /** @brief File path type
       *
       * The type used to represent a file path. The path is 
       * system agnostic which means that it should work on both 
       * posix and windows naming conventions.
       */
      typedef boost::filesystem::path path_type;

      /** @brief Log directory path
       *
       * This function gives the directory path where log_manager 
       * will store all the log files. If the directory was not 
       * set yet, it will both identify the correct path and create it.
       *
       * @throw SYSTEM_ERROR failed to create the log path
       * @return The target log directory
       * @post The log directory path returned is valid directory
       * @post log_manager log directory is inited
       */
      path_type log_path();
      /** @brief Set log directory path
       *
       * @param[in] path A directory name
       *
       * Set the log directory to path if it has not been set already.
       *
       * @pre log directory has not been inited yet
       *
       * @retval true if the operation succeeded
       * @retval false if the log directory has already been inited 
       *
       * @note this function do not check for the validity of its argument. 
       * The checking of the directory is made during the initalisation of 
       * the log_directory whcih is triggered by log_path() or nay other 
       * call that will trigger the creation of the log directory 
       * (such as log_file, cfg_path or syslog)
       *
       * @sa path_type log_path()
       */
      bool log_path(std::string const &path);
      
      /** @brief File path for a log file 
       *
       * @param[in] short_name A short file name
       *
       * Give thefull path for a file named @p short_name to be created in 
       * the log directory.  If @p short_name include a parent_path (for 
       * example "cfg/bar/foo") this call will also create the corresponding
       * directory structure in the log path (in that case it will create 
       * the directory cfg/bar). 
       * This method calls log_path() in order to identify the log directory.
       *
       * @throw SYSTEM_ERROR failed to create log directory or one of the
       * parent directories of @p short_name within th elog_directory. 
       *
       * @return The full path for @p short_name in the log directory
       *
       * @post log_manager log directory is inited
       * @post all the parent directories of @p file_name have been 
       * properly created within the log_directory
       */
      path_type log_file(std::string const &short_name);
      
      /** @brief Add directory to search path
       *
       * @param path A directory path
       *
       * Add @p path to the directories the log_manager will uses to 
       * locate files. @p poath can be absolute or relative (in which 
       * case this path will be searched relatively to the directory
       * where the program is currently running)
       *
       * @retval true if @p path has been succesfully added to the search path
       * @retval false if the path is not valid (ie not a directory)
       *
       * @post if the value returned is true then @path is now part of 
       * the search directory. 
       *
       * @note paths added through this called are appended at the end 
       * of the search list
       *
       * @sa search_path
       * @sa locate
       */
      bool add_search_path(std::string const &path);
      /** @brief search path
       *
       * Gives the gloabl search path the manager uses to locate files. 
       * 
       * @return A string gving all the paths used for locating files 
       * separated by columns
       *
       * @sa add_search_path
       * @sa locate
       */
      std::string search_path();

      /** @brief log directory for configuration files
       *
       * This methods give the target directory were all the configuration 
       * files used by trex can be stored. 
       *
       * This method is just a conveninence function and is requivalent to 
       * log_file("cfg")
       *
       * @return A path where all configurations files can be logged.
       * 
       * @sa log_file
       */
      path_type cfg_path() {
        return log_file("cfg");
      }
      /** @brief locate an existing file
       *
       * @param[in]  file_name  A file name
       * @param[out] found      success flag
       *
       * Try to locate @p file_name in the serahc path. The way log_manage 
       * locates a file is as follow:
       * @li check if @p file_name exists as is
       * @li if the above failed and @p file_name is not absolute check
       *  if this file exists on any directory in the search path
       * @note In order to consider the file as located it neeeds to be a
       * regular file oar a link to a regulare file (ie not a directory)
       *
       * @return if @p found is true then it returns a valid path 
       * for @p file_name otherswise it just retrun @p file_name
       * 
       * @sa search_path
       * @sa use
       */
      path_type locate(std::string const &file_name, bool &found);
      /** brief locate and log a file
       *
       * @param[in]  file_name  A file name
       * @param[out] found      success flag
       *
       * This function behaves similarly to @p locate except that in
       * addition it will copy @p file_name into the log config 
       * directory if @p file_name was located
       *
       * @return if @p found is true then it returns a valid path
       * for @p file_name otherwise it just retrun @p file_name
       *
       * @post if found is truen then @p file_name has now a loca copy 
       * under cfg_path()
       *
       * @sa locate
       * @sa cfg_path
       */
      path_type use(std::string const &file_name, bool &found);

      // TREX log
      /** @{
       * @brief Create a new log entry
       *
       * @param[in] when An optional time tag
       * @param[in] who  An optional specifier for the source of the message
       * @param[in] kind An optional specifier for the type of message
       *
       * Create a new log entry to be logged in the main TREX.log file. 
       *
       * @return The log entry that will recieve the message to be logged
       *
       * @sa TREX::utils::log::text_log
       */
      log::text_log &syslog();
      log::stream syslog(log::id_type const &who,
                         log::id_type const &kind=log::null) {
        return syslog()(who, kind);
      }
      log::stream syslog(log::entry::date_type const &when,
                         log::id_type const &who,
                         log::id_type const &kind=log::null) {
        return syslog()(when, who, kind);
      }
      /** @} */
      /** @brief Flush TREX.log
       *
       * Force the buffer of TREX.log file to be flushed into the file
       */
      void flush();
      
      template<typename Handler>
      boost::signals2::connection on_new_log(Handler fn) {
        return syslog().connect(fn);
      }
      template<typename Handler>
      boost::signals2::connection on_new_log(Handler fn,
                                             boost::asio::strand &s) {
        return syslog().connect(fn, s);
      }
      template<typename Handler>
      boost::signals2::connection on_new_log(Handler fn, bool) {
        return syslog().direct_connect(fn);
      }
      
      /** @brief Number of threads
       *
       * Indicate the number of thread managed by the pool of this log_manager.
       * All these threads are tied to the log_manager asio service
       *
       * @return the number of threads within the pool
       *
       * @sa thread_count(size_t, bool)
       * @sa service()
       */
      size_t thread_count() const;
      /** @brief Set thread_count
       *
       * @param[in] n minimum number of thread desired
       * @param[in] override_hw should we override the number of ahrdware threads ?
       *
       * Set the number of threads in the pool to at least @p n unless @p n 
       * is greater than the number of hardware threads and @p override_hw 
       * is @c false (the default). All these threads ewill then be attached 
       * to this log_manager asio service
       *
       * @return the number of thread in this pool after the operation
       *
       * @sa service()
       */
      size_t thread_count(size_t n, bool override_hw=false);
      /** @brief log_manager asio service
       *
       * The service this log_manager uses in all of its threads in order 
       * to execute asynchronous operations. The servie is boost asio 
       * io_service and can be used by any client to post asynchronous
       * operations that will then be executed within the thread_pool 
       * this class manages
       *
       * @return the io_service for this manager
       *
       * @sa thread_count()
       */
      boost::asio::io_service &service();
      
      
    private:
      log_manager();
      ~log_manager();
      
      UNIQ_PTR<details::mgmt_impl> m_impl;
      
      friend class singleton::wrapper<log_manager>;
    };
    
  } // TREX::utils 
} // TREX

#endif // H_LogManager

