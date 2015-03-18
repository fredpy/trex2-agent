# -*- cmake -*-
#######################################################################
# Software License Agreement (BSD License)                            #
#                                                                     #
#  Copyright (c) 2014, Universidade do Porto - Faculdade de           #
# Engenharia Laboratório de Sistemas e Tecnologia Subaquática (LSTS). #
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

cmake_minimum_required(VERSION 2.6.0 FATAL_ERROR)
project(hub_tracker)

#######################################################################
# CPack configuration
#######################################################################

set(PROJECT_VENDOR "Universidade do Porto - LSTS")
set(PROJECT_COPYRIGHT "Copyright (C) 2014 - ${PROJECT_VENDOR}")
set(PROJECT_MAJOR 0)
set(PROJECT_MINOR 1)
set(PROJECT_PATCH 0)

set(VERSION "${PROJECT_MAJOR}.${PROJECT_MINOR}.${PROJECT_PATCH}")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "hub tracker for iridium messages from trex")
set(CPACK_PACKAGE_VENDOR "${PROJECT_VENDOR}")
set(CPACK_PACKAGE_DESCRIPTION_FILE ${CMAKE_CURRENT_SOURCE_DIR}/README)
set(CPACK_PACKAGE_VERSION ${VERSION})
set(
CPACK_SOURCE_PACKAGE_FILE_NAME
"${CMAKE_PROJECT_NAME}-${VERSION}"
CACHE INTERNAL "Package basename"
)

set(CPACK_SOURCE_IGNORE_FILES
"~$"
"/\\\\.svn/"
"\\\\.DS_Store$"
)

include(CPack)

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

#######################################################################
# Look for boost library
#######################################################################

find_package(Boost 1.47.0 REQUIRED COMPONENTS system date_time thread)

if(NOT Boost_FOUND) 
  message(ERROR "Unable to find Boost (>=1.47)")
endif(NOT Boost_FOUND)

include_directories(${Boost_INCLUDE_DIRS})

#######################################################################
# Look for dune
#######################################################################

set(DUNE_HOME $ENV{DUNE_HOME} CACHE PATH "Expected location for dune")

find_library(DUNE_CORE_LIB NAMES dune-core HINTS ${DUNE_HOME}/lib
  DOCS "Looking for dune core library")

if(NOT DUNE_CORE_LIB)
  message(ERROR "Failed to locate dune-core library\n"
    "Please set tyur DUNE_HOME where dune is installed.")
endif(NOT DUNE_CORE_LIB)

find_path(DUNE_INCLUDE "DUNE/DUNE.hpp" HINTS ${DUNE_HOME}/include)

if(NOT DUNE_INCLUDE)
  message(ERROR "Failed to locate dune-core headers\n"
    "Please set tyur DUNE_HOME where dune is installed.")
endif(NOT DUNE_INCLUDE)

include_directories(${DUNE_INCLUDE})


#######################################################################
# Look for Wt
#######################################################################

set(Wt_HINT "/usr/local" CACHE PATH "Hint to Wt location")
set(CMAKE_MODULE_PATH "${Wt_HINT}/cmake;${Wt_HINT}/cmake/Modules/")

find_package(Wt)

#######################################################################
# Set up my project source
#######################################################################

include_directories(include)

add_library(hub_lib 
  # source
  src/client.cc
  src/message_handler.cc
  # headers
  include/client.hh
  include/message_handler.hh
)

target_link_libraries(hub_lib
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_DATE_TIME_LIBRARY}
  ${SYSTEM_LIBRARIES}
  ${DUNE_CORE_LIB})

add_executable(hub_tracker 
  # sources 
  src/main.cc
  src/server.cc
  src/msg_board.cc
  # headers
  include/server.hh
  include/msg_board.hh
)

target_link_libraries(hub_tracker hub_lib 
  ${Wt_LIBRARY} 
  ${Wt_HTTP_LIBRARY} 
  ${Boost_THREAD_LIBRARY})

install(TARGETS hub_tracker DESTINATION bin)
