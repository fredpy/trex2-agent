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
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake;${CMAKE_SOURCE_DIR}/cmake/Modules/")

########################################################################
# Project definitions                                                  #
########################################################################
project(trex)

set(TREX_MAJOR 0)
set(TREX_MINOR 6)
set(TREX_PATCH 0)
set(TREX_RC 1)

# setup the configuration for CPack
include(package_cfg)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  # Need this for config generated files
  ${CMAKE_BINARY_DIR}/trex/config
  )

# look for basic system libraries and configurations
include(system_cfg)

# look for boost or c++
include(boost_or_cpp11)


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
   configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in 
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

# TREX utilities functions for cmake compilation
include(trex_utils)

########################################################################
# TREX sub directories                                                 #
########################################################################

set(TREX_SHARED share/trex)

include_directories(${CMAKE_BINARY_DIR})

# core libraries 
add_subdirectory(trex)
trex_cfg(cfg ${TREX_SHARED})

# include extra/plugins
# add_subdirectory(extra) 

########################################################################
# Finalize                                                             #
########################################################################

include(trex_finalize)

get_property(TREX_INCS GLOBAL PROPERTY TREX_INCLUDES)
get_property(trex-core GLOBAL PROPERTY trex-core)
get_property(trex-extra GLOBAL PROPERTY trex-extra)
get_property(trex-python GLOBAL PROPERTY trex-python)
list(REMOVE_DUPLICATES TREX_INCS)
if(CPP11_ENABLED)
  set(trex_FLAGS ${CPP11_COMPILER_SWITCH})
endif(CPP11_ENABLED)
configure_file(${CMAKE_SOURCE_DIR}/cmake/trex-config.cmake.in 
  ${CMAKE_BINARY_DIR}/cmake/trex-config.cmake @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/cmake/trex-config.cmake 
  DESTINATION share/trex/cmake)
unset(TREX_INCS)

# All the packages are mixed together so older version of cmake are not confused
install(EXPORT trex-targets DESTINATION share/trex/cmake/imports OPTIONAL)

#include(CMakePackageConfigHelpers)
# Note I need to do a real version checking
#write_basic_package_version_file(trexConfigVersion.cmake VERSION ${VERSION} COMPATIBILITY AnyNewerVersion)

