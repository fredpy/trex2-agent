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
#ifndef H_trex_europa_Schema
# define H_trex_europa_Schema

# include <fstream>
# include <set>
# include <string>

# include <trex/utils/LogManager.hh>

# include <trex/europa/config.hh>

// include plasma header as system files in order to disable warnings
# define TREX_PP_SYSTEM_FILE <PLASMA/Debug.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/Error.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/CFunction.hh>
# include <trex/europa/bits/system_header.hh>

namespace TREX {
  namespace europa {
    
    class EuropaPlugin;
    class Assembly;

    namespace details {
      
      /** @brief Extension and logging management for Europa
       *
       * This singleton class is used internally in order to allow a reactor to:
       * - inject new Europa extensions that it can then manipulate
       * - redirect the debug output in its own specific file
       *
       * This class is not meant to be used directly except for Assembly (the europa 
       * connection of the EuropaReactor) or EuropaPlugin which allow to declare new 
       * europa extensions to be attached to reactors
       *
       * @author Frederic Py <fpy@mbari.org>
       */       
      class Schema :boost::noncopyable {
      public:
	void registerComponents(Assembly const &assembly);
        boost::asio::io_service &service() {
          return m_log->service();
        }
        std::string file_name(std::string const &path) {
          return m_log->file_name(path).string();
        }
	void setStream(std::ostream &out);
	
	std::string const &nddl_path();
        /** @brief recursive use for nddl
         * 
         * @param[in] file A file name
         * @param[out] found found indication flag
         *
         * This method locate the file @p file using TREX::utils::LogManager::use 
         * if the file was found it sets @p found to @c true and also try to 
         * locate all the files included by this file recursively and copy them 
         * in the @c cfg/ log directory 
         *
         * @retval The real path name of @p file if @p found is @c true 
         * @retval @p file oherwise
         *
         * @sa TREX::utils::LogManager::use
         * @sa include(std::istream &)
         */
        std::string use(std::string file, bool &found);
        
      private:
	Schema();
	~Schema() {}

        /** @brief Include files extraction
         * @param[in] input an input stream
         *
         * Extract all the file names included using a C like @c #include 
         * directive in @a input
         *
         * @return The set of all the file names included 
         *
         * @bug This is a simple implementation that do not check in depth 
         * if the @c #include is commented or not
         */ 
        static std::set<std::string> includes(std::istream &input);

	void registerPlugin(EuropaPlugin &pg);
	void unregisterPlugin(EuropaPlugin &pg);
        
        void registerFunction(Assembly const &assembly, EUROPA::CFunction *fn);

	std::set<EuropaPlugin *> m_plugins;
	std::ofstream            m_europa_debug;
	std::string              m_path;

	TREX::utils::SingletonUse<TREX::utils::LogManager> m_log;
	typedef std::map<std::string, boost::filesystem::path> 
          include_map;
        include_map m_includes;
        
        
	friend class TREX::utils::SingletonWrapper<Schema>;
	friend class TREX::europa::EuropaPlugin;
      }; // TREX::europa::details::Schema
      
    } // TREX::europa::details
  } // TREX::europa
} // TREX

#endif // H_trex_europa_Schema
