/** @file   Pdlfcn.hh
 * @author Patrick Daley
 * @date   
 * @brief Dynamic librares loading
 *
 * These functions provide a wrapper for unix @c dlopen(), @c dlsym(), 
 * @c dlclose() and @c dlerror() functions. When compiled for MAC OS X, these 
 * functions emulate the unix dl functions using Apple's API.
 *
 * @note Taken from Europa 
 * @ingroup utils
 */
#ifndef H_trex_utils_private_dl_func
#define H_trex_utils_private_dl_func

# include <private/dl_info.hh>

namespace trex {
  namespace utils {
    namespace internals {
			
      /** @brief dynamic library load
			 *
       * @param[in] path The library name
       * @param[in] mode The mode
       *
       * Open the dynamic library @p path with the mode @p mode
       *
       * @return A pointer to the dynamic library handle or @c NULL in case of 
			 *         failure
       *
       * @sa int p_dlclose(void *)
       * @sa const char *p_dlerror()
       * 
       * @author Patrick Daley
       * @ingroup utils
       */
      void *p_dlopen(const char *path, int mode);
			
      /** @brief dynamic library symbol access
			 * 
       * @param[in] handle the dynamic library handle
       * @param[in] symbol symbol to look for
       *
       * Look for the symbol @p symbol in the library @p handle
       *
       * @pre @p handle should be a valid handle as returned by @c p_dlopen
       *
       * @return A pointer to the entity corresponding to @p symbol or @c NULL 
			 *         in case of failure
       *
       * @sa void *p_dlopen(const char*, int)
       * @sa const char *p_dlerror()
       * @sa std::string const &p_dlext()
       * 
       * @author Patrick Daley
       * @ingroup utils
       */
      void * p_dlsym(void * handle, const char *symbol);
			
      /** @brief dynamic library error messages
       *
       * This methosds give the error message corresponding to the
       * error of the last call to a @p p_dl* function
       *
       * @return an error message
       *
       * @sa void *p_dlopen(const char*, int)
       * @sa void *p_dlsym(void *, const char *)
       * @sa int p_dlconst(void *)
       * 
       * @author Patrick Daley
       * @ingroup utils
       */
      const char * p_dlerror(void);
			
      /** @brief Close a dynamic library
			 *
       * @param[in,out] handle a library handle
       *
       * This method unload the library @p handle
       *
       * @retval 0 success
       * @retval -1 failure 
			 * 
       *
       * @sa void *p_dlopen(const char *, int)
       * @sa const char *p_dlerror()
       * 
       * @author Patrick Daley
       * @ingroup utils
       */
      int p_dlclose(void * handle);
			
      /** @brief Dynamic library extension helper
       *
       * This function helps to identify the usual extension of dynamic
       * libraries for the current platform. Indeed, depending on the OS this
       * extension differ. 
       *
       * The cases that are supported now are:
       * @li @c .so for linux and most of the UNIXes
       * @li @c .dylib under MacOS X
       * @li @c .dll under Windows
       *
       * @return The usual dynamic library extension  for this platform
       *
       * @sa void *p_dlopen(const char *, int)
       *
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup utils
       */
      inline char const *p_dlext() {
        return DL_LIB_EXTENSION;
      }
    }
  }
}

#endif // H_trex_utils_private_dl_func
