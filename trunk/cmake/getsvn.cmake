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

find_package(Subversion QUIET)
if(Subversion_FOUND)
  Subversion_WC_INFO(${SOURCE_DIR} MY)
  # tweak the url to be more compact
  string(REGEX REPLACE "^.*/svn/(.*)$" "\\1" MY_WC_PATH ${MY_WC_URL})
  set(MY_FLAG 1)
  # Now I need t osurun svnversion whihc is better than svn info
  find_program(SVN_VERSION_CMD svnversion 
    DOC "subversion version command")
  if(SVN_VERSION_CMD)
    execute_process(COMMAND ${SVN_VERSION_CMD} ${SOURCE_DIR} ${MY_SVN_TRUNK}
      OUTPUT_VARIABLE MY_WC_REVISION
      OUTPUT_STRIP_TRAILING_WHITESPACE)
  endif(SVN_VERSION_CMD)
else(Subversion_FOUND)
  set(MY_FLAG 0)
  set(MY_WC_PATH    "unknown")
  set(MY_WC_REVISION "exported")
endif(Subversion_FOUND)

file(WRITE svn_version.hh.txt 
  "#define SVN_INFO ${MY_FLAG}\n"
  "#define SVN_ROOT \"${MY_WC_PATH}\"\n"
  "#define SVN_REV \"${MY_WC_REVISION}\"\n"
)

execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different
  svn_version.hh.txt svn_version.hh)

