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

macro(trex_lib name)
  add_library(${name} SHARED)
  set_target_properties(${name} PROPERTIES OUTPUT_NAME "TREX${name}")
  add_dependencies(core ${name})
  install(
    TARGETS ${name} 
    EXPORT trex
    LIBRARY DESTINATION lib
  )
endmacro(trex_lib name)


# Core libraries 

add_subdirectory(utils)
add_subdirectory(domain)
add_subdirectory(transaction)
add_subdirectory(agent)

# pyhton support disbaled for now as it is incomplete anyway 
# TODO enable this with pybind11 later
#add_subdirectory(python)

# utility commands 
include(CheckFunctionExists)

check_function_exists(setpriority HAVE_SETPRIORITY)
if(NOT HAVE_SETPRIORITY)
  check_function_exists(nice HAVE_NICE)
endif(NOT HAVE_SETPRIORITY)

configure_file(cmds/nice_flags.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/nice_flags.h @ONLY)

#if(WITH_PYTHON)
#  include_directories(${PYTHON_INCLUDE_DIR})
#  set(extra_libs ${LINK_NO_AS_NEEDED} ${py_libs})
#else(WITH_PYTHON)
#  set(extra_libs "")
#endif(WITH_PYTHON)

add_executable(amc)
target_sources(amc PRIVATE
 cmds/AMC.cc
)
target_include_directories(amc PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(amc agent Boost::program_options)
add_dependencies(core amc)
install(
  TARGETS amc 
  EXPORT trex
  RUNTIME DESTINATION bin
)

#include_directories(${PYTHON_INCLUDE_DIR})

add_executable(sim)
target_sources(sim PRIVATE
  cmds/Sim.cc
)
target_link_libraries(sim agent Boost::program_options)
add_dependencies(core sim)
install(
  TARGETS sim
  EXPORT trex
  RUNTIME DESTINATION bin
)
