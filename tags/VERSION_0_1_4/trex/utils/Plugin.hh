/** @file "trex/utils/Plugin.hh"
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
      /** @brief Pointer to a plugin function
       * 
       * This type is used internally to know how to refer to the @c initPlugin 
       * function
       *
       * @sa TREX::initPlugin()
       * @ingroup utils
       */
      typedef void (*plugin_fn)();
    }
  }
} 

#endif 
