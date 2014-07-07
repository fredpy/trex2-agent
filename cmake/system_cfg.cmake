# -*- cmake -*- 
#######################################################################
# Software License Agreement (BSD License)                            #
#                                                                     #
#  Copyright (c) 2014, Frederic Py.                                   #
#  All rights reserved.                                               #
#                                                                     #
#  Redistribution and use in source and binary forms, with or without #
#  modification, are permitted provided that the following conditions #
#  are met:                                                           #
#                                                                     #
#   * Redistributions of source code must retain the above copyright  #
#     notice, this list of conditions and the following disclaimer.   #
#   * Redistributions in binary form must reproduce the above         #
#     copyright notice, this list of conditions and the following     #
#     disclaimer in the documentation and/or other materials provided #
#     with the distribution.                                          #
#   * Neither the name of the TREX Project nor the names of its       #
#     contributors may be used to endorse or promote products derived #
#     from this software without specific prior written permission.   #
#                                                                     #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS #
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT   #
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS   #
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE      #
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, #
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,#
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;    #
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER    #
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT  #
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN   #
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE     #
# POSSIBILITY OF SUCH DAMAGE.                                         #
#######################################################################

# older verision of Apple need special handling : this test the version of the system
if(APPLE)
  # -- Determine the version of OSX
  # -- 8 and less are OSX 10.0 - 10.4
  # -- 9 is 10.5 (LEOPARD)
  EXEC_PROGRAM(uname ARGS -v  OUTPUT_VARIABLE DARWIN_VERSION)
  STRING(REGEX MATCH "[0-9]+" DARWIN_VERSION ${DARWIN_VERSION})
  MESSAGE(STATUS "MacOS darwin version = ${DARWIN_VERSION}")
  IF (DARWIN_VERSION LESS 9)
    SET(APPLE_PRE_POSIX 1 INTERNAL)
  ENDIF (DARWIN_VERSION LESS 9)
endif(APPLE)


########################################################################
# Linux 64 really needs -fPIC                                          #
########################################################################
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-fPIC CXX_HAS_FPIC)
if(CXX_HAS_FPIC)
  # This is only required for Linux but we can have it anyway
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
  message (STATUS "Adding -fPIC to CXX flags")
else(CXX_HAS_FPIC)
  message (STATUS "CXX flags do not provide -fPIC")
endif(CXX_HAS_FPIC)

########################################################################
# System libraries needed by some system such as linux                 #
########################################################################
include(CheckLibraryExists)
include(CheckIncludeFileCXX)

if(WINDOWS OR MSVC)
  message(STATUS "windows system detected: TREX compilation is yet to be tested agains it")
  set(DL_TYPE "windows")
else(WINDOWS OR MSVC)
  set(DL_REQUIRED TRUE)
  if(APPLE AND APPLE_PRE_POSIX)
    check_include_file_cxx("mach-o/dyld.h" DYLD_FOUND)
    if(DYLD_FOUND)
      set(DL_TYPE "mach_o")
      set(DL_REQUIRED FALSE)
    else(DYLD_FOUND)
      message(WARNING "Failed to locate \"mach_o/dyld.h\" on an old version of macos\n"
	"I will attempt to locate libdl instead")
    endif(DYLD_FOUND)
  endif()
  
  if(DL_REQUIRED)
    check_library_exists(dl dlopen "" LIB_DL)
    if(LIB_DL)
      set(SYSTEM_LIBRARIES ${SYSTEM_LIBRARIES} dl)
      set(DL_TYPE "posix")
    else()
      message(FATAL_ERROR "TREX requires libdl on POSIX systems")
    endif(LIB_DL)
  endif(DL_REQUIRED)

  check_library_exists(pthread pthread_self "" LIB_PTHREAD)
  if(LIB_PTHREAD)
    set(SYSTEM_LIBRARIES ${SYSTEM_LIBRARIES} pthread)
  endif(LIB_PTHREAD)

  check_library_exists(rt shm_unlink "" LIB_RT)
  if(LIB_RT)
    set(SYSTEM_LIBRARIES ${SYSTEM_LIBRARIES} rt)
  endif(LIB_RT)
endif(WINDOWS OR MSVC)

string(TOUPPER ${DL_TYPE} UPPER_DL_TYPE)
set(DL_${UPPER_DL_TYPE} TRUE)
message(STATUS "Identified DL_TYPE to be ${DL_TYPE} (DL_${UPPER_DL_TYPE} = ${DL_${UPPER_DL_TYPE}})")

set(CMAKE_GENERATED_MESSAGE
  "DO NOT EDIT: File automatically generated by cmake.")
