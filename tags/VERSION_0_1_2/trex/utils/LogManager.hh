/** @file "utils/base/LogManager.hh"
 * @brief LogManager class definition
 * 
 * This header defines the LogManager utility alogn with several macro and 
 * functiona that eases file management in TREX
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */
#ifndef H_LogManager
# define H_LogManager

# include <list>

# include "TextLog.hh"
# include "SingletonUse.hh"
# include "SharedVar.hh"
# include "ErrnoExcept.hh"

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
 *       @c MAX_LOG_ATTEMPT missions in a single day. Ifndeed if one look 
 *       at the code of LogManager::createLatest he will see that it makes this 
 *       number of attempt stariung from the directory referred by @c LATEST_DIR
 *       link. Such behavior should really happen if the symbolic link 
 *       @c LATEST_DIR is not set properly. 
 *
 * @relates TREX::utils::LogManager
 * @author Frederic Py <fpy@mbari.org>
 */
# define MAX_LOG_ATTEMPT 1024

/** @brief Default verbosity level
 *
 * Specifies the level of verbosity to select if not given
 *
 * @relates TREX::utils::LogManager
 * @author Frederic Py <fpy@mbari.org>
 */
# define TREX_LOG_LEVEL LogNormal

namespace TREX {
  namespace utils {
		
    /** @brief Centralized logging and files management.
     * 
     * This class is a core component on TREX logging and file management. 
		 * It provides a transparent one-stop shopping to :
		 * @li Locate and load files
		 * @li produce text log messages toward a singfle log directory
		 * This class manages in a cntral way all this while ensuring that files 
		 * that are used by an agent are easily accessed and stored in a single log 
		 * directory giving the ability to knbow exactly what files were used on a 
		 * specific run. 
		 *
     * It provides utilities that will create a log directory and the
     * default TextLog file so other components can produce messages
     * seamlessly.
     *
     * It gives information of the user selected verbosity level which
     * can then be used by other components to identify whether they
     * should log this message or not.
     *
     * It also provide a way to put all the configurations files used
     * by the program in a specific directory.
     *
     * This class is a pure singleton and consequently its access should
     * be done using SingletonUse :
     * @code
     * #include "LogManager.hh"
     *
     * TREX::utils::SingletonUse<TREX::utils::LogManager> log;
     *
     * log->syslog("me")<<"A message to the log"<<std::endl;
     * @endcode
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    class LogManager :boost::noncopyable {
    public:
      
      /** @brief Log verbosity level */
      enum LogLevel {
				LogMin = 0,  //!< Minimum verbosity
				LogNormal,   //!< Normal level
				LogExtensive //!< High verbosity (usually for debugging)
      };
			
      /** @brief Logging path
       * This method indicates in which directory log files
       * should be placed. It also manage the creation of
       * this directory and other files to simplify its
       * organization/access
       *
       * @throw ErrnoExcept a problem occured while trying to create
       * log directories.
       *
       * @return a string with the full name of the log directory
       */
      std::string const &logPath();
      /** @brief log file complete name
       * @param[in] short_name A short file name
       *
       * This method indicates the full name of a file that has to be
       * stored in the log directory
       *
       * @throw ErrnoExcept A problem occured while trying to create log
       * directory
       *
       * @return The complete file name for the file @p short_name into
       * the log directory
       *
       * @sa logPath()
       */
      std::string file_name(std::string const &short_name);
      /** @brief Configuration files management
       * @param[in] file_name A configuration file name
       * @param[out] found Indicate if the file was found
       *
       * This method is used to indicate that @p file_name has been
       * used. It copies this file into the @c cfg subdirectory of
       * the log directory.
       *
       * If the file @p file_name cannot be found it also display
       * a message in the log file.
       *
       * @note This function uses @c locate method to look for the file
       * in the current  directory and the search path
       *
       * @throw  ErrnoExcept A problem occured while trying to copy @p file 
       * @throw  ErrnoExcept A problem occured while trying to create log
       * directory
       * @return A complete name of @p file_name along with the path where it 
			           was located 
       * @sa logPath()
       * @sa file_name(std::string const &)
       * @sa std::string locate(std::string const &, bool &) const
       */
      std::string use(std::string const &file_name, bool &found);
			
      /** @brief Named text entry
       * @param[in] who The name of the source
       *
       * This method create a new text entry on the TREX log.
       * If @p who is provided the entry will start with the
       * text @c [@p who].
       *
       * @return The text entry to be used with classical @c @<@<
       * output stream operator.
			 * @{
       */
      internals::LogEntry syslog(std::string const &who);
      TextLog &syslog();
			/** @} */

      /** @brief Current verbosity level
       * @return the current verbosity level
       * @sa setLogLevel(LogLevel)
       */
      LogLevel getLogLevel() const {
				return m_level;
      }
      /** @brief Set verbosity level
       * @param[in] lvl current verbosity level
       *
       * Sets the verbosity level to @p lvl
			 *
       * @pre The verbosity level cannot be changed after the
       * LogManager has been fully initialized
       *
       * @retval true level sucessfully set to @p lvl
       * @retval false the verbosity level cannot be changed
       * for this session
       *
       * @sa getLogLevel() const
       * @sa setLogPath(std::string const &)
       */
      bool setLogLevel(LogLevel lvl);
      /** @brief Set log path
       * @param[in] path A path
       *
       * Sets the  log directory to @p path
			 *
       * @pre The verbosity level cannot be changed after the
       * LogManager has been fully initialized
       *
       * @retval true log directory set to @p path
       * @retval false the log directory has already been created
       * and cannot be changed anymore
       *
       * @sa setLogLevel(LogLevel)
       */
      bool setLogPath(std::string const &path);
			
      /** @brief add a directory to the search path
       * @param path A directory
       *
       * This method adds @p path  to the end of the search path.
       * It will do so only if @p path is a valid directory or
       * is starting with @c '.'
       *
       * @retval true if @p path was successfully added
       * @retval false otherwise
       *
       * @post if the returned value is @c true @a path
       * is now part of the serch path
       *
       * @sa std::string locate(std::string const &file, bool &found) const
       */
      bool addSearchPath(std::string const &path);

      typedef std::list<std::string>::const_iterator path_iterator;
      path_iterator begin() const {
	return m_searchPath.begin();
      }
      path_iterator end() const {
	return m_searchPath.end();
      }
			
      /** @brief Locate a file 
       * @param[in] file   A file name
       * @param[out] found Indicator of success/failure
       *
       * This methods tries to locate @p file in the search path.
       * it first attempt to see if @p file exists by its own.
       * If it is not the case and @p file do not starts with a @c '/',
       * it iterates through the search path and tries to locate the file.
       * 
       * @retval @p file if @a found is @c false which indicates that it was 
			 *         unable to locate this file
       * @retval A valid symbolic name for @a file if @a found is @c true
       *
       * @sa bool addSearchPath(std::string const &path)
       * @sa std::string use(std::string const &, bool &)
       */
      std::string locate(std::string const &file, bool &found) const;
      
      /** @brief Get directory where cfg files should be stored
       *
       * @sa use(std::string const &)
       */
      std::string getCfgPath();
    private:
      /** @brief Log directory path */
      std::string     m_path;
			
      /** @brief Initialisation flag
       * This flag is set to true as soon as the log manager is
       * fully initialized
       */
      SharedVar<bool> m_inited;
      /** @brief syslog text log file */
      TextLog     m_syslog;
      /** @brief verbosity level */
      LogLevel    m_level;
			
      /** @brief Constructor */
      LogManager();
      /** @brief Destructor */
      ~LogManager();
			
      /** @brief latest symbolic link creation.
       * 
       * This method creates a symbolic link with the
       * name given by @c LATEST_DIR that refers to the
       * log directory. It also manage the creation of the log
       * directory if this one does not exist yet.
       *
       * @throw ErrnoExcept unable to create log directory
       */
      void createLatest();
      
      /** @brief load search path from environement variables
       *
       * This method is called during initialisation and load
       * the default search path from environment variables.
       *
       * @sa TREX_ENV
       * @sa SEARCH_ENV
       * @sa void addSearchPath(std::string const &)
       */
      void loadSearchPath();
			
      /** @brief Search path
       *
       * This varisble is built using @c TREX_ENV and @c SEARCH_ENV
       * and stores the set of directories to look for when trying
       * to locate a file
       *
       * @sa std::string locate(std::string const &file, bool &found) const
       */
      std::list<std::string> m_searchPath;
      
      friend class SingletonWrapper<LogManager>;
    }; // TREX::utils::LogManager
		
  } // TREX::utils 
} // TREX

#endif // H_LogManager
