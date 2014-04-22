/** @file "trex/utils/PluginLoader.cc"
 *
 * @brief Implementation of PluginLoader
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
#include "plugin_loader.hh"
#include "private/Pdlfcn.hh"
#include "Plugin.hh"

#include <boost/system/error_code.hpp>


using namespace TREX::utils;
using namespace TREX::utils::internals;

/*
 * class TREX::utils::plugin_loader
 */

// structors :

plugin_loader::~plugin_loader() {
  /*
   * Unloading plug-ins at destruction appeared to be a bad idea.
   * Indeed I have no guarantee that objects from ythe loaded 
   * library are dangling. 
   */
  // handle_map::iterator i;
  
  // // Unload all the plugins loaded 
  // while( !m_loaded.empty() ) {
  //   i = m_loaded.begin();
  //   void *handle = i->second.first;
  
  //   if( 0!=p_dlclose(handle) )
  //     throw PluginError(i->first, "Failed to unload");
  //   m_loaded.erase(i);
  // }
}

// Modifiers :

bool plugin_loader::load(symbol const &name,
                        bool fail_on_locate) {
  handle_map::iterator i = m_loaded.find(name);
  if( m_loaded.end()==i ) {
    bool found;
    std::string const libName = "lib"+name.str()+p_dlext();
    std::string fileName = m_log->locate(libName, found).string();
    if( found ) {
      if( libName==fileName )
	fileName = "./"+libName;
      m_log->syslog("plugin", log::info)<<"Loading "<<fileName;
      void *handle = p_dlopen(fileName.c_str(), RTLD_NOW);
      if( NULL==handle ) {
        ERROR_CODE ec = make_error_code(ERRC::executable_format_error);
        throw SYSTEM_ERROR(ec, "Failed to load library "+name.str());
      }
      plugin_fn f_init = (plugin_fn)p_dlsym(handle, "initPlugin");
      if( NULL==f_init ) {
        ERROR_CODE ec = make_error_code(ERRC::function_not_supported);
        SYSTEM_ERROR err(ec, "plugin("+name.str()+") Function initPLugin missing");
        p_dlclose(handle);
        throw err;
      }
      m_loaded[name] = std::make_pair(handle, 1);
      // call to f_init : may throw an exception but
      //                  it is too be avoided
      f_init();
    } else {
      if( fail_on_locate ) {
        ERROR_CODE ec = make_error_code(ERRC::no_such_file_or_directory);
        throw SYSTEM_ERROR(ec, "Failed to locate plugin "+name.str());
      }
      return false;
    }
  } else {
    i->second.second += 1;
  }
  return true;
}

bool plugin_loader::unload(symbol const &name) {
  handle_map::iterator i = m_loaded.find(name);
  if( m_loaded.end()!=i ) {
    if( (i->second.second--)<=1 ) {
      void *handle = i->second.first;
      if( 0!=p_dlclose(handle) ) {
        ERROR_CODE ec = make_error_code(ERRC::io_error);
        throw SYSTEM_ERROR(ec, "Failed to unload plugin("+i->first.str()+")");
      }
      m_loaded.erase(i);
      return true;
    }
  }
  return false;
}
