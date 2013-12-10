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

cmake_minimum_required(VERSION 2.8)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")

project(trex_turtle)

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "TREX ROS integration example")
set(CPACK_PACKAGE_VENDOR "TREX2 development team")
set(CPACK_PACKAGE_DESCRIPTION_FILE ${CMAKE_CURRENT_SOURCE_DIR}/README)
set(CPACK_GENERATOR "TGZ" CACHE STRING "Packaging method for binary")
set(
CPACK_SOURCE_PACKAGE_FILE_NAME
"${CMAKE_PROJECT_NAME}"
CACHE INTERNAL "tarball basename"
)
set(CPACK_SOURCE_GENERATOR "TGZ" CACHE STRING "Packaging method for source")
set(CPACK_SOURCE_IGNORE_FILES
"~$"
"/\\\\.svn/"
"\\\\.DS_Store$"
)

include(CPack)



# set up trex
find_package(trex REQUIRED)
include_directories(${trex_INCLUDE_DIRS})
add_definitions(${trex_CPPFLAGS})

# set up ros
find_package(ROS)
if(ROS_CONFIG)
  find_ros_pkg(roscpp)

  include_directories(${ROS_roscpp_INCLUDE_DIRS})
  add_definitions(${ROS_roscpp_CFLAGS})
  link_directories(${ROS_roscpp_LINK_PATH})

  find_ros_pkg(go_turtle)
  if(ROS_go_turtle_PACKAGE_PATH)
    set(EXTRA_SRC src/nav_adapter.cc)
    include_directories(${ROS_go_turtle_INCLUDE_DIRS})
    add_definitions(${ROS_go_turtle_CFLAGS})
    link_directories(${ROS_go_turtle_LINK_PATH})
  else(ROS_go_turtle_PACKAGE_PATH)
    message(WARNING "go_turlte ROS package not detected") 
  endif(ROS_go_turtle_PACKAGE_PATH)
else(ROS_CONFIG)
  message(SEND_ERROR "Failed to localize ROS")
endif(ROS_CONFIG)

# Define my new trex plugin
add_library(turtle_pg SHARED 
  src/turtle_pg.cc
  src/odom_adapter.cc
  src/pose_adapter.cc
  src/fail_guard.cc
  ${EXTRA_SRC})

target_link_libraries(turtle_pg TREXros)

# Make sure you install this guy in TREX_PATH
install(TARGETS turtle_pg DESTINATION share/trex/plugins)
install(DIRECTORY cfg DESTINATION share/trex OPTIONAL
  FILES_MATCHING PATTERN "*"
  PATTERN "*~" EXCLUDE
  PATTERN ".svn" EXCLUDE)

