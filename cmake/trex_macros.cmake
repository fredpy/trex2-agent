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

include(ide_structure)

########################################################################
# TREX related macros                                                  #
########################################################################
macro(trex_cfg dir dest)
  install(DIRECTORY ${dir} DESTINATION ${dest} OPTIONAL
    FILES_MATCHING PATTERN "*")
  set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_CFGS 
    ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
endmacro(trex_cfg)

function(expand_libs target)
  get_property(deps TARGET ${target} PROPERTY LINK_LIBRARIES)
  if(deps)
    foreach(lib ${deps})
      if(TARGET ${lib})
	get_property(d2 TARGET ${lib} PROPERTY LINK_LIBRARIES)
	if(d2)
	  list(REMOVE_ITEM d2 ${deps})
	  target_link_libraries(${target} ${d2})
	endif(d2)
      endif(TARGET ${lib})
    endforeach()
  endif(deps)
endfunction(expand_libs)
  

function(trex_lib target kind)
  set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_LIBS 
    ${CMAKE_CURRENT_BINARY_DIR})
  get_property(tmp TARGET ${target} PROPERTY LINK_DEPENDS)
  set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_LIBS 
    ${tmp})
  expand_libs(${target})

  if(CPP11_ENABLED) 
    set_target_properties(${target} PROPERTIES LINK_FLAGS ${CPP11_LINK_FLAGS})
  endif(CPP11_ENABLED)

  # transmit directory include to target include for older cmake
  get_property(tmp DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
  set_property(TARGET ${target} APPEND PROPERTY INCLUDE_DIRECTORIES ${tmp})
  unset(tmp)

  trex_organize_target(${target})
  
  if(kind) 
    # update global include path for installed trex

    get_property(incs TARGET ${target} PROPERTY INCLUDE_DIRECTORIES)
    if(incs)
      # cleanup 
      list(REMOVE_DUPLICATES incs)
      set(i_incs ${CMAKE_INSTALL_PREFIX}/include)
      foreach(i ${incs})
	if(NOT ${i} MATCHES ${CMAKE_SOURCE_DIR})
	  if(NOT ${i} MATCHES ${CMAKE_BINARY_DIR})
	    list(APPEND i_incs ${i})
	  endif(NOT ${i} MATCHES ${CMAKE_BINARY_DIR})
	endif(NOT ${i} MATCHES ${CMAKE_SOURCE_DIR})
      endforeach()
      if(i_incs)
	list(REMOVE_DUPLICATES i_incs)
      endif(i_incs)
      unset(incs)
      set_property(GLOBAL APPEND PROPERTY TREX_INCLUDES ${i_incs})
      unset(i_incs)
    endif(incs)

    # message(STATUS "trex-${kind} += ${target}")
    install(TARGETS ${target} DESTINATION lib 
      EXPORT trex-targets)
    set_property(GLOBAL APPEND PROPERTY trex-${kind} ${target})
  endif(kind)
endfunction(trex_lib)

function(trex_py target)
  set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_PYTHON ${CMAKE_CURRENT_BINARY_DIR})
  expand_libs(${target})
  trex_organize_target(${target})
  
  if(CPP11_ENABLED) 
    set_target_properties(${target} PROPERTIES LINK_FLAGS ${CPP11_LINK_FLAGS})
  endif(CPP11_ENABLED)
  install(TARGETS ${target} DESTINATION ${TREX_SHARED}/python OPTIONAL EXPORT trex-targets) 
  set_property(GLOBAL APPEND PROPERTY trex-python ${target})
endfunction(trex_py)

macro(trex_cmd target)
  set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_CMDS
    ${CMAKE_CURRENT_BINARY_DIR})
  expand_libs(${target})
  trex_organize_target(${target})
  
  if(CPP11_ENABLED) 
    set_target_properties(${target} PROPERTIES LINK_FLAGS ${CPP11_LINK_FLAGS})
  endif(CPP11_ENABLED)
  get_property(tmp TARGET ${target} PROPERTY LINK_DEPENDS)
  set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_LIBS 
    ${tmp})
  unset(tmp)
endmacro(trex_cmd)

macro(bash_path var property) 
  get_property(tmp GLOBAL PROPERTY ${property})
  if(tmp)
    list(REMOVE_DUPLICATES tmp)
  endif(tmp)
  string(REPLACE ";" ":" ${var} "${tmp}")
  unset(tmp)
endmacro(bash_path)
