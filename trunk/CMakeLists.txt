# -*- cmake -*- 
#######################################################################
# Software License Agreement (BSD License)                            #
#                                                                     #
#  Copyright (c) 2011, MBARI.                                         #
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

cmake_minimum_required(VERSION 2.8)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/")

########################################################################
# Project definitions                                                  #
########################################################################
project(trex)
set(TREX_MAJOR 0)
set(TREX_MINOR 3)
set(TREX_PATCH 3)


# CPack version numbers for release tarball name.
set(CPACK_PACKAGE_VERSION_MAJOR ${TREX_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${TREX_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${TREX_PATCH})

set(VERSION "${TREX_MAJOR}.${TREX_MINOR}.${TREX_PATCH}")
message(STATUS "${PROJECT_NAME} VERSION = ${VERSION}")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "TREX agent executive")
set(CPACK_PACKAGE_VENDOR "TREX2 development team")
set(CPACK_PACKAGE_DESCRIPTION_FILE ${CMAKE_CURRENT_SOURCE_DIR}/README)
set(CPACK_GENERATOR TGZ)
set(
CPACK_SOURCE_PACKAGE_FILE_NAME
"${CMAKE_PROJECT_NAME}-${VERSION}"
CACHE INTERNAL "tarball basename"
)
set(CPACK_SOURCE_GENERATOR TGZ)
# The following components are regex's to match anywhere (unless anchored)
# in absolute path + filename to find files or directories to be excluded
# from source tarball.
set(CPACK_SOURCE_IGNORE_FILES
"~$"
"log/latest/"
"log/[1-9][0-9]*\\\\.[0-9][0-9]*\\\\.[0-9][0-9]*/"
"/\\\\.svn/"
"\\\\.DS_Store$"
)
include(CPack)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  )

########################################################################
# System libraries                                                     #
########################################################################
include(CheckLibraryExists)

check_library_exists(dl dlopen "" LIB_DL)
if(LIB_DL)
  set(SYSTEM_LIBRARIES ${SYSTEM_LIBRARIES} dl)
else(LIB_DL) 
  message(FATAL_ERROR "TREX requires lib dl")  
endif(LIB_DL)

check_library_exists(pthread pthread_self "" LIB_PTHREAD)
if(LIB_PTHREAD)
  set(SYSTEM_LIBRARIES ${SYSTEM_LIBRARIES} pthread)
endif(LIB_PTHREAD)

check_library_exists(rt shm_unlink "" LIB_RT)
if(LIB_RT)
  set(SYSTEM_LIBRARIES ${SYSTEM_LIBRARIES} rt)
endif(LIB_RT)

########################################################################
# Boost C++ Libraries                                                  #
########################################################################
find_package(Boost 1.41.0 COMPONENTS thread system signals chrono)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  list(APPEND ${PROJECT_NAME}_LIBRARY_DIRS ${Boost_LIBRARY_DIRS})
else(Boost_FOUND)
  message(FATAL_ERROR "TREX requires Boost C++ libraries")
endif(Boost_FOUND)

########################################################################
# Europa                                                               #
########################################################################

option(WITH_EUROPA "Enable Europa plugin." ON) 

if(WITH_EUROPA) 
  find_package(Europa
    COMPONENTS PlanDatabase ConstraintEngine TemporalNetwork Utils TinyXml 
    NDDL RulesEngine Solvers System)
  if(NOT EUROPA_FOUND)
    message(FATAL_ERROR "Failed to find Europa while WITH_EUROPA is ON")
  endif(NOT EUROPA_FOUND)
endif(WITH_EUROPA)

########################################################################
# Doxygen                                                              #
########################################################################
find_package(Doxygen)
if(DOXYGEN_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in 
    ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
  set(DOXYGEN_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/doc CACHE STRING 
    "Output directory for doxygen")
  mark_as_advanced(DOXYGEN_OUTPUT)
  add_custom_target(doc
    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
endif(DOXYGEN_FOUND)

########################################################################
# TREX related macros                                                  #
########################################################################
macro(trex_cfg dir dest)
  install(DIRECTORY ${dir} DESTINATION ${dest} OPTIONAL
    FILES_MATCHING PATTERN "*"
    PATTERN ".svn" EXCLUDE)
  set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_CFGS 
    ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
endmacro(trex_cfg)

macro(trex_lib target)
  set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_LIBS 
    ${CMAKE_CURRENT_BINARY_DIR})
  get_property(tmp TARGET ${target} PROPERTY LINK_DEPENDS)
  set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_LIBS 
    ${tmp})
  unset(tmp)
endmacro(trex_lib)

macro(trex_cmd target)
  set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_CMDS
    ${CMAKE_CURRENT_BINARY_DIR})
  get_property(tmp TARGET ${target} PROPERTY LINK_DEPENDS)
  set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_LIBS 
    ${tmp})
  unset(tmp)
endmacro(trex_cmd)

macro(bash_path var property) 
  get_property(tmp GLOBAL PROPERTY ${property})
  list(REMOVE_DUPLICATES tmp)
  string(REPLACE ";" ":" ${var} "${tmp}")
  unset(tmp)
endmacro(bash_path)

########################################################################
# TREX sub directories                                                 #
########################################################################

# core libraries 
add_subdirectory(trex)
trex_cfg(cfg shared/trex)

# include extra/plugins
add_subdirectory(extra) 

# add your sub directories here if needed
add_subdirectory(mbari) # MBARI specific components

########################################################################
# Finalize                                                             #
########################################################################

bash_path(TREX_CONFIG_DIRS ${PROJECT_NAME}_CFGS)
bash_path(TREX_LIBRARY_DIRS ${PROJECT_NAME}_LIBS)
bash_path(TREX_BINARY_DIRS ${PROJECT_NAME}_CMDS)

# A file just for testing inside the build directory
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/trex_devel.bash.in
  ${CMAKE_CURRENT_BINARY_DIR}/trex_devel.bash @ONLY)

# create a default bash environement file for installed version
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/trex_init.bash.in
  ${CMAKE_CURRENT_BINARY_DIR}/trex_init.bash @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/trex_init.bash DESTINATION shared/trex)

# create the default log directory
install(DIRECTORY log DESTINATION shared/trex/log
   DIRECTORY_PERMISSIONS WORLD_READ WORLD_WRITE
   FILES_MATCHING PATTERN "*" EXCLUDE)


