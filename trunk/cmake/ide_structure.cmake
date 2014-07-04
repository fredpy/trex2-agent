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

macro(trex_organize_target target)
  get_target_property(src ${target} SOURCES)
  string(LENGTH ${CMAKE_CURRENT_SOURCE_DIR} src_len)
  string(LENGTH ${CMAKE_CURRENT_BINARY_DIR} bin_len)
      

  foreach(file ${src})
    get_filename_component(dir ${file} PATH)
    get_filename_component(ext ${file} EXT)

    if(dir)
      string(LENGTH ${dir} dir_len)
      if(dir_len GREATER ${src_len})
	string(SUBSTRING ${dir} 0 ${src_len} tmp)

	if(tmp STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
	  math(EXPR len "${dir_len} - ${src_len}")
	  string(SUBSTRING "${dir}" ${src_len} ${len} dir)
	endif()
      elseif(dir_len GREATER ${bin_len})
	string(SUBSTRING ${dir} 0 ${bin_len} tmp)

	if(tmp STREQUAL ${CMAKE_CURRENT_BINARY_DIR})
	  math(EXPR len "${dir_len} - ${bin_len}")
	  string(SUBSTRING "${dir}" ${bin_len} ${len} dir)
	endif()
      endif()
    endif(dir)
    if(dir)
      string(REPLACE "/" "\\" dir ${dir})
    endif(dir)

    # Check if header
    string(REGEX MATCH "^\\.h" header ${ext})
    if(header) 
      if(dir)
	source_group("Header\ Files\\${dir}" FILES ${file})
      endif(dir)
    else(header)
      string(REGEX MATCH "^\\.c" source ${ext})
      if(source) 
	if(dir)
	  source_group("Source\ Files\\${dir}" FILES ${file})
	endif(dir)
      else(source)
	string(REGEX MATCH "^\\.tcc$" template ${ext})
	if(template)
	  message(STATUS "Template ${target}:${file} to ${dir}"
	    "Source: ${tmp}")
	  source_group("Template\ Files\\${dir}" FILES ${file})
	else(template)
	  source_group("Other\ Files\\${dir}" FILES ${file})	  
	endif(template)
      endif(source)
    endif(header)
  endforeach(file ${src})
  
  

endmacro(trex_organize_target)