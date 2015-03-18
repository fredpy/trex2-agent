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

# indicate the cmake version we need
cmake_minimum_required(VERSION 2.8)
# tell cmake that some files it may need can be found here
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")

# Name of the project
project(trex_turtle)

# some extra things to easilly generate a tarball from the source or binaries 
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
# end of the extra things

#----------------- T-REX ----------------------------
# locate trex libraries and set up
find_package(trex REQUIRED)

# add the location of trex headers to the include path 
include_directories(${trex_INCLUDE_DIRS})
# add trex comilation flags to the compilation flags
add_definitions(${trex_CPPFLAGS})

#----------------- ROS ------------------------------ 
# locate ros
find_package(ROS)

if(ROS_CONFIG)
  # find the ros package roscpp
  find_ros_pkg(roscpp)

  # add include paths, compilations flags and libraries needed 
  # to use roscpp
  include_directories(${ROS_roscpp_INCLUDE_DIRS})
  add_definitions(${ROS_roscpp_CFLAGS})
  link_directories(${ROS_roscpp_LINK_PATH})

  # find the ros package go_turtle
  find_ros_pkg(go_turtle)
  if(ROS_go_turtle_PACKAGE_PATH)
    # add the source files dedicated for comm with go_turtle 
    set(EXTRA_SRC src/nav_adapter.cc)

    # add compile flags for go_turtle 
    include_directories(${ROS_go_turtle_INCLUDE_DIRS})
    add_definitions(${ROS_go_turtle_CFLAGS})
    link_directories(${ROS_go_turtle_LINK_PATH})

  else(ROS_go_turtle_PACKAGE_PATH)

    # go_turtle was not found: print a warning
    message(WARNING "go_turlte ROS package not detected") 
  endif(ROS_go_turtle_PACKAGE_PATH)
else(ROS_CONFIG)
  # ROS was not found: send an error
  message(SEND_ERROR "Failed to localize ROS")
endif(ROS_CONFIG)


#------------------------- My trex plugin -----------------------

# Define my new trex plugin
add_library(turtle_pg SHARED 
  src/turtle_pg.cc          # this file defines the plugin API TREX uses (a function that T_REX neeed to find on any plugin)
  src/odom_adapter.cc       # define conversion from ros odometry messages to a trex token
  src/pose_adapter.cc       # define conversion from ros pose...  messages to a trex token
  src/fail_guard.cc         # define a simple reactor that will kill trex if a reactor is killed
  ${EXTRA_SRC}              # extra code if go_turtle was found: adapter from go_turtle topics to trex tokens
)

target_link_libraries(turtle_pg TREXros)

# Make sure you install this guy in TREX_PATH
install(TARGETS turtle_pg DESTINATION share/trex/plugins) # install the plugin libarary here 

install(DIRECTORY cfg DESTINATION share/trex OPTIONAL # install configuration files (the content of cfg) there
  FILES_MATCHING PATTERN "*"
  PATTERN "*~" EXCLUDE
  PATTERN ".svn" EXCLUDE)

