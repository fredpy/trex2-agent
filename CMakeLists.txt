########################################################################
# Copyright (c) 2011, Ricardo Martins                                  #
# All rights reserved.                                                 #
#                                                                      #
# Redistribution and use in source and binary forms, with or without   #
# modification, are permitted provided that the following conditions   #
# are met:                                                             #
#                                                                      #
# Redistributions of source code must retain the above copyright       #
# notice, this list of conditions and the following disclaimer.        #
# Redistributions in binary form must reproduce the above copyright    #
# notice, this list of conditions and the following disclaimer in the  #
# documentation and/or other materials provided with the distribution. #
#                                                                      #
# Neither the name of the Universidade do Porto - Faculdade de         #
# Engenharia nor the names of its contributors may be used to endorse  #
# or promote products derived from this software without specific      #
# prior written permission.                                            #
#                                                                      #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS  #
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT    #
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS    #
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE       #
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, #
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, #
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;     #
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER     #
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT   # 
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN    #
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE      #
# POSSIBILITY OF SUCH DAMAGE.                                          #
########################################################################

cmake_minimum_required(VERSION 2.8)

########################################################################
# Project definitions                                                  #
########################################################################
project(trex)
set(TREX_MAJOR 0)
set(TREX_MINOR 1)
set(TREX_PATCH 2)

########################################################################
# Boost C++ Libraries                                                  #
########################################################################
find_package(Boost COMPONENTS system)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
else(Boost_FOUND)
  message(FATAL_ERROR "TREX requires Boost C++ libraries")
endif(Boost_FOUND)

########################################################################
# System libraries                                                     #
########################################################################
include(CheckLibraryExists)

check_library_exists(pthread pthread_self "" LIB_PTHREAD)
if(LIB_PTHREAD)
  set(SYSTEM_LIBRARIES ${SYSTEM_LIBRARIES} pthread)
endif(LIB_PTHREAD)

check_library_exists(rt clock_gettime "" LIB_RT)
if(LIB_RT)
  set(SYSTEM_LIBRARIES ${SYSTEM_LIBRARIES} rt)
endif(LIB_RT)

check_library_exists(dl dlopen "" LIB_DL)
if(LIB_DL)
  set(SYSTEM_LIBRARIES ${SYSTEM_LIBRARIES} dl)
endif(LIB_DL)

########################################################################
# Include Directories                                                  #
########################################################################
include_directories(
  ${PROJECT_SOURCE_DIR}
  trex/transaction
  trex/domain
  trex/agent
  trex/utils
  trex/rapidxml
  )

########################################################################
# Compiler definitions                                                 #
########################################################################
add_definitions(
  -DTREX_MAJOR=${TREX_MAJOR}
  -DTREX_MINOR=${TREX_MINOR}
  -DTREX_RELEASE=${TREX_PATCH}
  -DSUFSHARE="${CMAKE_SHARED_LIBRARY_SUFFIX}"
  )

########################################################################
# TREX libraries.                                                      #
########################################################################

# TREXutils
file(GLOB_RECURSE TREX_UTILS_FILES trex/utils/*.cc)
add_library(TREXutils SHARED ${TREX_UTILS_FILES})

# TREXdomain
file(GLOB_RECURSE TREX_DOMAIN_FILES
  RELATIVE ${PROJECT_SOURCE_DIR} trex/domain/*.cc)
add_library(TREXdomain SHARED ${TREX_DOMAIN_FILES})
target_link_libraries(TREXdomain TREXutils)

# TREXtransaction
file(GLOB_RECURSE TREX_TRANSACTION_FILES
  RELATIVE ${PROJECT_SOURCE_DIR} trex/transaction/*.cc)
add_library(TREXtransaction SHARED ${TREX_TRANSACTION_FILES})
target_link_libraries(TREXtransaction TREXdomain TREXutils)

# TREXagent
file(GLOB_RECURSE TREX_AGENT_FILES 
  RELATIVE ${PROJECT_SOURCE_DIR} trex/agent/*.cc)
add_library(TREXagent SHARED ${TREX_AGENT_FILES})
target_link_libraries(TREXagent TREXtransaction TREXdomain TREXutils)


set(TREX_LIBRARIES TREXutils TREXagent TREXdomain TREXtransaction)

########################################################################
# TREX plugins.                                                        #
########################################################################
macro(trex_plugin folder)
  get_filename_component(basename ${folder} NAME_WE)
  set(plugin_name "TREX${basename}")
  set(plugin_files "")
  file(GLOB_RECURSE plugin_files
    RELATIVE ${PROJECT_SOURCE_DIR} ${folder}/*.cc)
  add_library(${plugin_name} SHARED ${plugin_files})
  target_link_libraries(${plugin_name}
    ${TREX_LIBRARIES} ${Boost_LIBRARIES} ${SYSTEM_LIBRARIES})
endmacro(trex_plugin)

trex_plugin(extra/vitre)
trex_plugin(extra/lightswitch)
if(EUROPA)
  trex_plugin(extra/europa)
endif(EUROPA)

########################################################################
# TREX executables.                                                    #
########################################################################
macro(trex_cmd path)
  get_filename_component(basename ${path} NAME_WE)
  string(TOLOWER ${basename} cmd_name)
  add_executable(${cmd_name} ${path})
  target_link_libraries(${cmd_name}
    ${TREX_LIBRARIES} ${Boost_LIBRARIES} ${SYSTEM_LIBRARIES})
endmacro(trex_cmd)

trex_cmd(trex/cmds/AMC.cc)
trex_cmd(trex/cmds/Sim.cc)
