#ifndef H_TREXversion
# define H_TREXversion

# include <string> 

namespace TREX {
    
	/** @brief Core libraies version information
	 * 
	 * This simple class lows to access to information about the current TREX 
	 * version of TREX core libararies
	 * 
	 * @ingroup utils
	 * @author Frederic Py <fpy@mbari.org>
	 */
  struct version {
		/** @brief version number
		 * 
		 * Indicates the version number of this library. This version number follows 
		 * the same nomenclature as th boost versionning and is guaranteed to 
		 * increase as we have new versions. 
		 * 
		 * @return 100*(100*major()+minor())+release()
		 * @sa major()
		 * @sa minor()
		 * @sa release()
		 * @sa str()
		 */
    static unsigned long number();

		/** @brief Major version number
		 * @return the major version number
		 */
    static unsigned short major();
		/** @brief Minor version number
		 * @return the minor version number
		 */
    static unsigned short minor();
		/** @brief Patch version number
		 * @return the patch version number
		 */
    static unsigned short release();

		/** @brief Human readable version
		 * 
		 * Indicates the version number as a human readable string
		 * the string is formatted the usual way :
		 * @<major@>.@<minor@>.@<patch@>
		 *
		 * @return The string value for this version
		 * @sa major()
		 * @sa minor()
		 * @sa release()
		 * @sa number()
		 */
    static std::string str();
    
  }; // TREX::version
  
} // TREX

#endif // H_TREXversion
