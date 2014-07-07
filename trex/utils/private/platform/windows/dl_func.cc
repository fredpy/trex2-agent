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
#include <windows.h>

namespace {
  DWORD dlerror_last = ERROR_SUCCESS;
}

void *TREX::utils::internals::p_dlopen(const char *path, int mode) {
  if(!path) {
    dlerror_last = ERROR_NOT_SUPPORTED;
    return NULL;
  }
  HMODULE result = LoadLibrary(path);
  
  if(!result)
    dlerror_last = GetLastError();
  return result;
}

void *TREX::utils::internals::p_dlsym(void * handle, const char *symbol) {
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

const char *TREX::utils::internals::p_dlerror(void) {
  if(dlerror_last != ERROR_SUCCESS) {
    static char buffer[256];
    if(!FormatMessage(FORMAT_MESSAGE_IGNORE_INSERTS | FORMAT_MESSAGE_FROM_SYSTEM, NULL, 
		      dlerror_last, 0, buffer, sizeof(buffer), NULL))
      snprintf(buffer, sizeof(buffer), "Failed to format error message");

    dlerror_last = ERROR_SUCCESS;
    
    return buffer;
  }
  return NULL;
}

int TREX::utils::internals::p_dlclose(void * handle) {
  HMODULE hmodule = (HMODULE) handle;
  
  if(!hmodule)
    return 0;
  
  int result = FreeLibrary(hmodule);
  if(!result)
    dlerror_last = GetLastError();
  
  return !result;
}
