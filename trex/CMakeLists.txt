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
#     from this software without specific prior written permmission.   #
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

# pyhton support disbaled for now as it is incomplete anyway 
add_subdirectory(python)

# utility commands 
include(CheckFunctionExists)

check_function_exists(setpriority HAVE_SETPRIORITY)
if(NOT HAVE_SETPRIORITY)
  check_function_exists(nice HAVE_NICE)
endif(NOT HAVE_SETPRIORITY)

configure_file(cmds/nice_flags.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/nice_flags.h @ONLY)

if(WITH_PYTHON)
  include_directories(${PYTHON_INCLUDE_DIR})
  set(extra_libs "${LINK_NO_AS_NEEDED}${py_libs}")
else(WITH_PYTHON)
  set(extra_libs "")
endif(WITH_PYTHON)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_executable(amc cmds/AMC.cc)
target_link_libraries(amc TREXagent ${Boost_PROGRAM_OPTIONS_LIBRARY} ${extra_libs})
add_dependencies(core amc)
install(TARGETS amc DESTINATION bin)

trex_add_path_filter(amc cmds)
trex_cmd(amc)

include_directories(${PYTHON_INCLUDE_DIR})

add_executable(sim cmds/Sim.cc)
target_link_libraries(sim TREXagent ${Boost_PROGRAM_OPTIONS_LIBRARY} ${extra_libs})
add_dependencies(core sim)
install(TARGETS sim DESTINATION bin)

trex_add_path_filter(sim cmds)
trex_cmd(sim)
