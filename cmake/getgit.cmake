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
find_package(Git QUIET)
# Now I need to extract the project version info
message("Looking for git: ${GIT_FOUND}")

if(GIT_FOUND)
  # Now I need to extract the project version info
  message("git found: ${GIT_EXECUTABLE}")
  # extract git branch
  execute_process(COMMAND ${GIT_EXECUTABLE} describe --all
    WORKING_DIRECTORY ${SOURCE_DIR}
    RESULT_VARIABLE GIT_RET
    OUTPUT_VARIABLE MY_WC_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  message(STATUS "git describe -> ${GIT_RET} : \"${MY_WC_BRANCH}\"")
  
  if(GIT_RET EQUAL 0)
    # extract version number and info 
    execute_process(COMMAND ${GIT_EXECUTABLE} describe --always --dirty
      WORKING_DIRECTORY ${SOURCE_DIR}
      RESULT_VARIABLE GIT_RET
      OUTPUT_VARIABLE MY_WC_REV
      OUTPUT_STRIP_TRAILING_WHITESPACE)
  endif(GIT_RET EQUAL 0)
  
  if(NOT GIT_RET EQUAL 0)
    set(MY_FLAG 0)
    set(MY_WC_BRANCH "")
    set(MY_WC_REV "not_from_git")
  else(NOT GIT_RET EQUAL 0)
    set(MY_FLAG 1)
  endif(NOT GIT_RET EQUAL 0)
  
else(GIT_FOUND)
    set(MY_FLAG 0)
    set(MY_WC_BRANCH "")
    set(MY_WC_REV "git_not_found")
endif(GIT_FOUND)

file(WRITE git_version.hh.txt
  "/* DO NOT EDIT: File automatically generated */\n\n"
  "#define GIT_INFO ${MY_FLAG}\n"
  "#define GIT_BRANCH \"${MY_WC_BRANCH}\"\n"
  "#define GIT_REV \"${MY_WC_REV}\"\n"
)

message(STATUS "Git version: [${MY_WC_BRANCH}:${MY_WC_REV}]")

execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different
  git_version.hh.txt ${OUTPUT_DIR}/git/version.hh)