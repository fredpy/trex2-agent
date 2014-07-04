# -*- cmake -*- 
#######################################################################
# Software License Agreement (BSD License)                            #
#                                                                     #
#  Copyright (c) 2014, Frederic Py                                    #
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

option(ASIO_DEBUG "enable debug of asio" OFF)
mark_as_advanced(ASIO_DEBUG)

# Files generated based on the setup 
configure_file(generated/asio_conf.hh.in
  ${CMAKE_CURRENT_BINARY_DIR}/bits/asio_conf.hh)
configure_file(generated/cpp11.hh.in
  ${CMAKE_CURRENT_BINARY_DIR}/bits/cpp11.hh)
configure_file(generated/version.hh.in
  ${CMAKE_CURRENT_BINARY_DIR}/bits/version.hh)

# I need to make this target visible in Xcode and all 
add_custom_target(TREXconfig ALL SOURCES 
  chrono.hh
  cpp11_deleted.hh
  memory.hh
  system_error.hh
  version.hh
  ${CMAKE_CURRENT_BINARY_DIR}/bits/asio_conf.hh
  ${CMAKE_CURRENT_BINARY_DIR}/bits/cpp11.hh
  ${CMAKE_CURRENT_BINARY_DIR}/bits/version.hh)
trex_organize_target(TREXconfig)
  


install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bits 
  DESTINATION include/trex/config/
  FILES_MATCHING PATTERN "*.hh" PATTERN "*.tcc")

install(DIRECTORY . DESTINATION include/trex/config
  FILES_MATCHING PATTERN "*.hh" PATTERN "*.tcc"
  PATTERN "private" EXCLUDE
  PATTERN "generated" EXCLUDE
  PATTERN ".svn" EXCLUDE)

