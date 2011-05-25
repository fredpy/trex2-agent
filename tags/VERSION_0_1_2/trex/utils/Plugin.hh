/** @file "utils/base/Plugin.hh"
 *
 * @brief Plugin header
 *
 * This header defines the basic requirements for defining a plug-in. It 
 * provides the basic defintions necessary for loading a plugin. 
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils plugins
 */
#ifndef H_Plugin 
# define H_Plugin 

namespace TREX {
	
  /** @brief Method to be implemented for each plugin
   * @ingroup utils
   */
  extern "C" void initPlugin();
	
  namespace utils {
    namespace internals {
      typedef void (*plugin_fn)();
    }
  }
} 

#endif 
