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
#include "PluginLoader.hh"
#include "private/Pdlfcn.hh"
#include "Plugin.hh"

using namespace TREX::utils;
using namespace TREX::utils::internals;

/*
 * class TREX::utils::PluginError
 */ 

PluginError::PluginError(Symbol const &name, 
                         std::string const &msg) throw()
:Exception("Plugin("+name.str()+") "+msg+": "+p_dlerror()) {}

/* 
 * class TREX::utils::PluginLoader
 */

// structors :

PluginLoader::~PluginLoader() {
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

void PluginLoader::load(Symbol const &name) {
  handle_map::iterator i = m_loaded.find(name);
  if( m_loaded.end()==i ) {
    bool found;
    std::string fileName = m_log->locate("lib"+name.str()+p_dlext(), found).string();
    if( found ) {
      m_log->syslog("plugin")<<"Loading "<<fileName;
      void *handle = p_dlopen(fileName.c_str(), RTLD_NOW);
      if( NULL==handle )
        throw PluginError(name, "Failed to load \""+name.str()+"\"");
      plugin_fn f_init = (plugin_fn)p_dlsym(handle, "initPlugin");
      if( NULL==f_init ) {
        PluginError err(name, "missing initPlugin");
        p_dlclose(handle);
        throw err;
      }
      m_loaded[name] = std::make_pair(handle, 1);
      // call to f_init : may throw an exception but
      //                  it is too be avoided
      f_init();
    } else 
      throw Exception("Unable to locate plugin "+name.str());
  } else {
    i->second.second += 1;
  }
}

bool PluginLoader::unload(Symbol const &name) {
  handle_map::iterator i = m_loaded.find(name);
  if( m_loaded.end()!=i ) {
    if( (i->second.second--)<=1 ) {
      void *handle = i->second.first;
      if( 0!=p_dlclose(handle) )
        throw PluginError(i->first, "Failed to unload");
      m_loaded.erase(i);
      return true;
    }
  }
  return false;
}
