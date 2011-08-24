# -*- cmake -*-
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

configure_file(version.hh.in
  ${CMAKE_CURRENT_BINARY_DIR}/version.hh @ONLY)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/version.hh 
  DESTINATION include/trex
)

add_custom_target(core COMMENT "TREX core libraries")

# Core libraries 

add_subdirectory(utils)
add_subdirectory(domain)
add_subdirectory(transaction)
add_subdirectory(agent)

# utility commands 

add_executable(amc cmds/AMC.cc)
target_link_libraries(amc TREXagent)
add_dependencies(core amc)
install(TARGETS amc DESTINATION bin)

add_executable(sim cmds/Sim.cc)
target_link_libraries(sim TREXagent)
add_dependencies(core sim)
install(TARGETS sim DESTINATION bin)