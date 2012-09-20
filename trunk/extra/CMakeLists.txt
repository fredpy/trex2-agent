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

add_custom_target(plugins COMMENT "TREX plugins")

macro(trex_plugin name src)
  add_library(${name}_pg SHARED ${src} ${ARGN})
  add_dependencies(plugins ${name}_pg)
  install(TARGETS ${name}_pg DESTINATION ${TREX_SHARED}/plugins OPTIONAL)
  trex_cfg(cfg ${TREX_SHARED})
  set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_CFGS 
    ${CMAKE_CURRENT_BINARY_DIR})
  trex_lib(${name}_pg)
endmacro(trex_plugin)

# the directories where plugins are located

add_subdirectory(witre.old)
# add_subdirectory(witre)


add_subdirectory(backward)
add_subdirectory(lightswitch) 
add_subdirectory(europa)
add_subdirectory(rover)
add_subdirectory(shopping)

option(WITH_LSTS "Use LSTS plugin" OFF)
# mark_as_advanced(WITH_LSTS)
if(WITH_LSTS)
  add_subdirectory(lsts)
endif(WITH_LSTS)

option(WITH_NUS "Use NUS plugin" OFF)
if(WITH_NUS)
  add_subdirectory(starfish)
endif(WITH_NUS)
