/**  @file trex/utils/Pdlfcn.cc
 * @author Patrick Daley
 * @brief unix @c dl* wrappers to support different platforms
 *
 * These functions provide a wrapper for unix @c dlopen(), @c dlsym(),
 * @c dlclose(), and @c dlerror() functions. When compiled for
 * @c __APPLE__, these functions emulate the unix function using Apple's API.
 * 
 * @note OS-X 10.4 (Tiger) introduced a native Apple implementation of 
 * these functions with GCC 4.0
 *
 * @ingroup utils
 */
#include "Pdlfcn.hh"

#if defined(__APPLE__) && (__GNUC__ < 4)   
#include <mach-o/dyld.h>
#elif defined(__MINGW32__)
#include <windows.h>
#else
#include <dlfcn.h>
#endif
#include<iostream>
#include<stdlib.h>

#define DLERROR_STR_MAX_LEN 256

#ifndef SUFSHARE
# error "SUFSHARE needs to be defined to allow compilation"
#endif

namespace TREX {
  namespace utils {
    namespace internals {

std::string const &p_dlext() {
  static std::string const ext(SUFSHARE);
  return ext;
}

#if defined(__APPLE__) && (__GNUC__ < 4)    
namespace {
  /*
   * Provide portable dl functions using Apple's API for gcc prior to v4
   */
  
	/** @brief error conmessage construction helper
	 * 
	 * @param[in] isSet A flag 
	 * @param[in] fmt   A typical @c *printf formatting string
	 * 
	 * This function is an helper to build or extract an error message produced 
	 * during a @c dl_* operation. Its behavior depends on the value of the flag 
	 * @p isSet:
	 * @li when @p isSet is @c true, it will ostore the new message as specified 
	 *     by @p fmt and following arguments
	 * @li when @p isSet is @c false, it will extract the last message produced.
	 * 
	 * @note the message can be read only once. if a call is made with @p isSet 
	 *       @c false. The message won't be anymore available. I am not sure about 
	 *       the validity of such behavior in general but as these functions are 
	 *       not meant to be used directly it should be fine. 
	 *
	 */
  static const char *setError(int isSet, const char *fmt, ...) {
    static char errorStr[DLERROR_STR_MAX_LEN];
    static int errorIsSet = 0;
    va_list arg; 
    if (isSet) {
      //set the error string
      va_start(arg, fmt);
      vsnprintf(errorStr, sizeof(errorStr), fmt, arg);
      va_end(arg);
      errorIsSet = 1;
    } else {
      //get the error string
      if (errorIsSet) {
        errorIsSet = 0;
        return errorStr;
      } else {
        return 0;
      }
    }
    return 0;
  }
}

void * p_dlopen(const char *path, int mode) {  /* mode ignored */
  NSObjectFileImage *fileImage;
  NSModule handle = 0;
  NSObjectFileImageReturnCode  returnCode =
    NSCreateObjectFileImageFromFile(path, &fileImage);
  
  if(returnCode == NSObjectFileImageSuccess) {
    //try to load bundle (not used for EUROPA2)
    handle = NSLinkModule(fileImage, path,
			  NSLINKMODULE_OPTION_RETURN_ON_ERROR |
			  NSLINKMODULE_OPTION_PRIVATE);
  } else if(returnCode == NSObjectFileImageInappropriateFile) {
    //try to load dynamic library (normal path in EUROPA2)
    handle = NSAddImage(path, NSADDIMAGE_OPTION_RETURN_ON_ERROR);
  }
  if (!handle) {
    setError(1, "p_dlopen:  Could not load shared library: %s", path);
  }
  return handle;
}

void * p_dlsym(void * handle, const char * symbol) {
  char ubSymbol[256];
  snprintf (ubSymbol, sizeof(ubSymbol), "_%s", symbol);
  
  NSSymbol nssym = NSLookupSymbolInImage(handle, ubSymbol,
					 NSLOOKUPSYMBOLINIMAGE_OPTION_RETURN_ON_ERROR |
					 NSLOOKUPSYMBOLINIMAGE_OPTION_BIND);
  if (!nssym) {
    setError(1, "p_dlsym: Did not find symbol %s", ubSymbol);
  }
  return (NSAddressOfSymbol(nssym));
}


const char * p_dlerror(void) {
  return setError(0, (char *) 0);
}

int p_dlclose(void * handle) {
  
  DYLD_BOOL result = NSUnLinkModule(handle, 0);
  if (!result) {
    setError(1, "p_dlclose: Failed to close library %s", NSNameOfModule(handle));
    //darwin doesn't seem to support unloading shared libraries, so fake it.
    return 0;
  }
  return 0;
}
#elif defined(__MINGW32__)

namespace {
  DWORD dlerror_last = ERROR_SUCCESS;
}
  
void* p_dlopen(const char* path, int mode) {
  if(!path) {
    dlerror_last = ERROR_NOT_SUPPORTED;
    return NULL;
  }
  HMODULE result = LoadLibrary(path);
  
  if(!result)
    dlerror_last = GetLastError();
  return result;
}

void* p_dlsym(void* handle, const char* symbol) {
  HMODULE hmodule = (HMODULE) handle;
  
  if(!handle) {
    dlerror_last = ERROR_INVALID_HANDLE;
    return NULL;
  }
  
  void* result = (void*) GetProcAddress(hmodule, symbol);
  
  if(!result)
    dlerror_last = GetLastError();
  
  return result;
}

const char* p_dlerror(void) {
  if(dlerror_last != ERROR_SUCCESS) {
    static char buffer[256];
    if(!FormatMessage(FORMAT_MESSAGE_IGNORE_INSERTS | FORMAT_MESSAGE_FROM_SYSTEM, NULL, dlerror_last, 0, buffer, sizeof(buffer), NULL))
      snprintf(buffer, sizeof(buffer), "Failed to format error message");

    dlerror_last = ERROR_SUCCESS;
    
    return buffer;
  }
  return NULL;
}

int p_dlclose(void* handle) {
  HMODULE hmodule = (HMODULE) handle;
  
  if(!hmodule)
    return 0;
  
  int result = FreeLibrary(hmodule);
  if(!result)
    dlerror_last = GetLastError();
  
  return !result;
}
#else
/*
 * use unix dl functions in all non Apple cases
 */
void * p_dlopen(const char *path, int mode) {  
  return dlopen(path, mode);
}

void * p_dlsym(void * handle, const char * symbol) {
  return dlsym(handle, symbol);
}

const char * p_dlerror(void) {
  return dlerror();
}

int p_dlclose(void * handle) {
  return dlclose(handle);
}
#endif
    } // TREX::utils::internals
  } // TREX::utils
} // TREX
