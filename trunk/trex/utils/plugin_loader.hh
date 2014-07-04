/* -*- C++ -*- */
/** @file "trex/utils/PluginLoader.hh"
 * @brief plug-in loading utility
 *
 * This file defines the class PluginLoader which is used to load TREX plug-ins
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
#ifndef H_trex_utils_plugin_loader
# define H_trex_utils_plugin_loader

# include <boost/unordered_map.hpp>

# include "symbol.hh"
# include "log_manager.hh"

namespace TREX {
  namespace utils {
		
    /** @brief plug-in management class
     *
     * This class implement a simple way to dynamically load TREX 
     * plug-ins.
     *
     * A TREX plug-in is named @c lib@<plugin@>.@<ext@> with @c
     * @<plugin@> is a plug-in symbolic identifier and @<ext@> is 
     * the platform specific extension for dynamic libraries. A 
     * TREX plug-in is also expected to provide the function: 
     * @code
     * extern "C" void TREX::initPlugin()
     * @endcode
     * as defined in the utils/base/Plugin.hh header and can 
     * ususally implements extensions of TREX such as new reactor 
     * or clock types.
     *
     * A plug-in can be located at any place wich is in the TREX 
     * search path as maintined by LogManager.
     *
     * This class is a pure singleton using some kind of reference 
     * counting to ensure that a plug-in won't be unloaded before 
     * it is not used anymore. Indeed, to unload a plug-in one need 
     * to either :
     * @li destroy the log_player singleton
     * @li or call @c unload for this plug-in as many times as he 
     * called @c load
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    class plugin_loader :public boost::noncopyable {
    public:
      /** @brief plug-in load method
       *
       * @param[in] name Name of the plug-in
       * @param[in] fail_on_locate falg for exception on 
        *   failure to locate
       *
       * This method attempts to load the plug-in @p name by 
       * searching for the file named @c lib@<name@>.@<ext@> 
       * in the search path provided by LogManager. 
       *
       * If it fails to lcoate this library it will either throw 
       * an Exception -- if @p fail_on_locate is @c true -- or 
       * just return @c false.  
       *
       * When this library is loaded it then attempts to call 
       * the @c initPlugin function of this plug-in
       *
       * This operation is guaranteed to be called only if the 
       * plug-in @p name is not currently loaded
       *
       * @throw SYSTEM_ERROR Problem while trying to load the
       *     plug-in @p name
       *
       * @retval true if the plug-in was successfully loaded 
       * @retval false if we failed to cloate @p name and 
       *    @p fail_on_locate is @c false
       */
      bool load(symbol const &name,
                bool fail_on_locate=true);
      /** @brief plug-in unload method
       *
       * @param[in] name Name of the plug-in
       *
       * If this method is called as many time as @c load for 
       * @p name. It will then unload this plug-in
       *
       * @throw SYSTEM_ERROR unable to unload the plug-in @a name
       *
       * @retval true The plug-in was unloaded
       * @retval false the plug-in was not unloaded. It can be
       *    because either this plug-in was not loaded on the 
       *    first place or is still in use. 
       */
      bool unload(symbol const &name);
      
    private:
      /** @brief Constructor */
      plugin_loader() {}
      /** @brief Destructor
       *
       * @note this destructor is @b not unloading the plug-ins 
       *   loaded as it could result on the program crashing if 
       *   one critical plug-in is unloaded before its code is 
       *   still needed.
       */
      ~plugin_loader();
			
      /** @brief Loaded plug-in storage type
       * 
       * This type is used internally to store the list of plug-in 
       * currently loaded by this class. 
       * 
       * It is a simple map that associates the symbolic name of 
       * a plug-in to a pointer to its handle and a reference
       * counter.
       */
      typedef boost::unordered_map<symbol, std::pair<void *, size_t> > handle_map;
			
      /** @brief Loaded plug-ins information
       * 
       * This attribute stores all the information needed for 
       * managing the plug-ins currnelty loaded
       */
      handle_map m_loaded;
      /** @brief LogManager entry point
       * 
       * This is an entry point to the TREX LogManager singleton. 
       * The LogManager here is not really used for logging 
       * information but mostly to uses the @c LogManager::locate 
       * method in order to locate the plug-in 
       */ 
      singleton::use<log_manager> m_log;
      
      friend class singleton::wrapper<plugin_loader>;
    }; // class TREX::utils::PluginLoader
		
		
  } // TREX::utils 
} // TREX

#endif // H_trex_utils_plugin_loader
