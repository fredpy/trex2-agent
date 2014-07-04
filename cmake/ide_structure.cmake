# -*- cmake -*- 
#######################################################################
# Software License Agreement (BSD License)                            #
#                                                                     #
#  Copyright (c) 2014, Frederic Py.                                   #
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
include(CMakeParseArguments)

macro(truncate_dir var dir base)
  set(result "${dir}")
  if(IS_DIRECTORY "${base}")
    string(LENGTH "${dir}" dir_len)
    string(LENGTH "${base}" base_len)
  
    if(dir_len GREATER ${base_len})
      string(SUBSTRING ${dir} 0 ${base_len} tmp)
      if(tmp STREQUAL "${base}")
	math(EXPR len "${dir_len} - ${base_len}")
	string(SUBSTRING ${dir} ${base_len} ${len} result)
      endif()
    endif()
  endif()
  set(${var} "${result}")
endmacro(truncate_dir)


macro(trex_organize_target target)
  get_target_property(src ${target} SOURCES)
       
  foreach(file ${src})
    if(NOT IS_DIRECTORY ${file})
      get_filename_component(dir ${file} PATH)
      get_filename_component(ext ${file} EXT)

      if(dir)
	truncate_dir(short_dir ${dir} ${CMAKE_CURRENT_SOURCE_DIR})
	if(short_dir STREQUAL ${dir})
	  truncate_dir(short_dir ${dir} ${CMAKE_CURRENT_BINARY_DIR})
	endif()
	set(dir "${short_dir}")
      endif(dir)
      if(dir)
	string(REPLACE "/" "\\" dir ${dir})
      endif(dir)

      # Check if header
      string(REGEX MATCH "^\\.h" header "${ext}")
      if(header) 
	if(dir)
	  source_group("Header\ Files\\${dir}" FILES ${file})
	endif(dir)
      else(header)
	string(REGEX MATCH "^\\.c" source "${ext}")
	if(source) 
	  if(dir)
	    source_group("Source\ Files\\${dir}" FILES ${file})
	  endif(dir)
	else(source)
	  string(REGEX MATCH "^\\.tcc$" template "${ext}")
	  if(template)
	    source_group("Template\ Files\\${dir}" FILES ${file})
	  else(template)
	    source_group("Other\ Files\\${dir}" FILES ${file})	  
	  endif(template)
	endif(source)
      endif(header)
    endif()
  endforeach(file ${src})
endmacro(trex_organize_target)