# -*- cmake -*-
########################################################################
# Copyright (c) 2011, LSTS                                             #
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

### Locate dune 
# NOTE fpy: dune home is set to this directory by default to look 
# for the dune local copy ... need to be changed when 
# the platform is not linux
set(DUNE_HOME "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "Home for the Dune library")

find_library(DUNE_CORE_LIB NAMES dune-core HINTS ${DUNE_HOME}/lib 
  DOCS "Looking for dune core library")
if(NOT DUNE_CORE_LIB)
  message(SEND_ERROR "Failed to locate dune-core library\n"
    "Set your DUNE_HOME where dune is installed.")
endif(NOT DUNE_CORE_LIB)

find_path(DUNE_INCLUDE_DIR "Dune/Dune.hpp" HINTS ${DUNE_HOME}/include)
if(NOT DUNE_INCLUDE_DIR)
  message(SEND_ERROR "Failed to locate dune headers\n"
    "Set your DUNE_HOME where dune is installed.")
endif(NOT DUNE_INCLUDE_DIR)

include_directories(${DUNE_INCLUDE_DIR})
include_directories(${EUROPA_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/extra/europa)

trex_plugin(platform 
  # source
  ControlInterface.cc 
  DuneClock.cc
  EuropaExtensions.cc 
  Platform.cc 
  SafetyBug.cc
  ImcAdapter.cc

  # headers
  ControlInterface.hh 
  DuneClock.hh
  EuropaExtensions.hh 
  Platform.hh 
  SafetyBug.hh
  ImcAdapter.hh
)

target_link_libraries(platform_pg TREXagent
  TREXeuropa_core
  ${Boost_LIBRARIES}
  ${DUNE_CORE_LIB}
)
