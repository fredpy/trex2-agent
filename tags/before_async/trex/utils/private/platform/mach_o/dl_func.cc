/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Frederic Py
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
#include <private/dl_func.hh>

#include <cstdarg>
#include <cstdio>

#include <mach-o/dyld.h>

# define DL_ERR_MAX_LEN 256

namespace {
  
  char const *set_error(bool is_set, char const *fmt, ...) {
    static char error_str[DL_ERR_MAX_LEN];
    static bool is_err_set = false;
    va_list     arg;
    
    if( is_set ) {
      va_start(arg, fmt);
      vsnprintf(error_str, sizeof(error_str), fmt, arg);
      va_end(arg);
      is_err_set = true;
    } else if( is_err_set ) {
      is_err_set = false;
      return error_str;
    }
    return 0;
  }

}

void *TREX::utils::internals::p_dlopen(const char *path, int mode) {
  NSObjectFileImage *file_img;
  NSModule handle = 0;
  NSObjectFileImageReturnCode ret = NSCreateObjectFileImageFromFile(path,
                                                                    &file_img);
  if( NSObjectFileImageSuccess==ret ) {
    // try to load a bundle (not used by trex plugin)
    handle = NSLinkModule(file_img, path,
			  NSLINKMODULE_OPTION_RETURN_ON_ERROR |
			  NSLINKMODULE_OPTION_PRIVATE);
  } else if( NSObjectFileImageInappropriateFile==ret ) {
    // try to load dynamic library (normal situation of trex)
    handle = NSAddImage(path, NSADDIMAGE_OPTION_RETURN_ON_ERROR);
  }
  if( !handle ) 
    set_error(true, "p_dlopen: could not load library: %s", path);
  return handle;
}

void *TREX::utils::internals::p_dlsym(void * handle, const char *symbol) {
  char ub_symbol[256];
  snprintf(ub_symbol, sizeof(ub_symbol), "_%s", symbol);
  
  NSSymbol ns_sym = NSLookupSymbolInImage(handle, ub_symbol,
					  NSLOOKUPSYMBOLINIMAGE_OPTION_RETURN_ON_ERROR |
					  NSLOOKUPSYMBOLINIMAGE_OPTION_BIND);
  if( !ns_sym ) 
    set_error(1, "p_dlsym: Unable to locate symbol %s in library", symbol);
  return (NSAddressOfSymbol(ns_sym));
}

const char *TREX::utils::internals::p_dlerror(void) {
  return set_error(0, NULL);
}

int TREX::utils::internals::p_dlclose(void * handle) {
  DYLD_BOOL result = NSUnLinkModule(handle, 0);
  if( !result ) {
    set_error(1, "p_dlclose: Failed to close library %s", NSNameOfModule(handle));
    // Old darwin did not support unloading modules so fake it anyway
    // return 1;
  }
  return 0;
}
