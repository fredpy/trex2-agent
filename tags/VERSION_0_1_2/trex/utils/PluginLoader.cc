/** @file "utils/base/PluginLoader.cc"
 *
 * @brief Implementation of PluginLoader
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */
#include "PluginLoader.hh"
#include "Pdlfcn.hh"
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
   * Unloading plug-ins at destruction appeared to be a bad idea
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
    std::string fileName = m_log->locate("lib"+name.str()+p_dlext(), found);
    if( found ) {
      void *handle = p_dlopen(fileName.c_str(), RTLD_NOW);
      if( NULL==handle )
				throw PluginError(name, "Failed to load \""+name.str()+"\"");
      plugin_fn f_init = (plugin_fn)p_dlsym(handle, "initPlugin");
      if( NULL==f_init ) {
				PluginError err(name, "missing initPugin");
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
