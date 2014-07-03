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

set(VERSION "${TREX_MAJOR}.${TREX_MINOR}.${TREX_PATCH}")

if(TREX_RC GREATER 0)
  set(VERSION "${VERSION}-rc${TREX_RC}")
endif(TREX_RC GREATER 0)

# CPack version numbers for release tarball name.
set(CPACK_PACKAGE_VERSION ${VERSION})

message(STATUS "${PROJECT_NAME} VERSION = ${VERSION}")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "TREX agent executive")
set(CPACK_PACKAGE_VENDOR "TREX2 development team")
set(CPACK_PACKAGE_DESCRIPTION_FILE ${CMAKE_CURRENT_SOURCE_DIR}/README)
set(CPACK_GENERATOR "TGZ" CACHE STRING "Packaging method for binary")
set(
CPACK_SOURCE_PACKAGE_FILE_NAME
"${CMAKE_PROJECT_NAME}-${VERSION}"
CACHE INTERNAL "tarball basename"
)
set(CPACK_SOURCE_GENERATOR "TGZ" CACHE STRING "Packaging method for source")
# The following components are regex's to match anywhere (unless anchored)
# in absolute path + filename to find files or directories to be excluded
# from source tarball.
set(CPACK_SOURCE_IGNORE_FILES
"~$"
"log/latest/"
"log/[1-9][0-9]*\\\\.[0-9][0-9]*\\\\.[0-9][0-9]*/"
"/\\\\.svn/"
"\\\\.DS_Store$"
)



set(TREX_EXTRA_SRC pkg)
set(TREX_FROM_PKG OFF)

if(IS_DIRECTORY ${CMAKE_SOURCE_DIR}/${TREX_EXTRA_SRC})
  # This code is 
  set(TREX_FROM_PKG ON)
  add_custom_target(trex_prep)
else()
  set(TREX_EXTRA_DIR ${CMAKE_BINARY_DIR}/${TREX_EXTRA_SRC}
    CACHE INTERNAL "Directory where extra generated source can be put")
  set(CPACK_SOURCE_INSTALLED_DIRECTORIES
    "${CMAKE_SOURCE_DIR};/;${TREX_EXTRA_DIR};/${TREX_EXTRA_SRC}")
  add_custom_target(trex_prep)
endif()

include(CPack)
