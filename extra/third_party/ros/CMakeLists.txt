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

# locate ROS
set(ROS_ROOT $ENV{ROS_ROOT} CACHE PATH 
  "root path for ROS")

option(ROS_NO_CHECK "Include without checking" OFF)
mark_as_advanced(ROS_NO_CHECK)

if(ROS_NO_CHECK)
  message(STATUS "Include ROS in the project with no checking")
  message(WARNING "Including ROS with no checking result on the "
    "project not being compilable")
else(ROS_NO_CHECK)
  if(NOT OLD_ROS_ROOT EQUAL ROS_ROOT)
    find_file(ROS_CMAKE rosbuild.cmake HINTS ${ROS_ROOT}/core/rosbuild)
    mark_as_advanced(ROS_CMAKE)
    if(ROS_CMAKE)
      set(ROS_CFG ${ROS_CMAKE} CACHE FILE "ROS configuration file" FORCE)
      mark_as_advanced(ROS_CFG)
      message(STATUS "ROS config found: ${ROS_CFG}")
    else(ROS_CMAKE)
      message(SEND_ERROR "Unable to find rosbuild.cmake")
    endif(ROS_CMAKE)
    set(OLD_ROS_ROOT ${ROS_ROOT} CACHE INTERNAL "former ROS_ROOT value" FORCE)
  endif(NOT OLD_ROS_ROOT EQUAL ROS_ROOT)
endif(ROS_NO_CHECK)

if(ROS_CFG)
  include(${ROS_CFG})

  # Now I need to check all the things I need  
  rosbuild_find_ros_package(roscpp)
  if(roscpp_PACKAGE_PATH)
    message(STATUS "Looking for roscpp: ${roscpp_PACKAGE_PATH}")
    # gets compilation information
    rosbuild_invoke_rospack(roscpp roscpp INCLUDE_DIRS cflags-only-I)
    rosbuild_invoke_rospack(roscpp roscpp CFLAGS cflags-only-other)
    rosbuild_invoke_rospack(roscpp roscpp LINK_PATH libs-only-L)
    rosbuild_invoke_rospack(roscpp roscpp LINK_LIBS libs-only-l)
    rosbuild_invoke_rospack(roscpp roscpp LINK_FLAGS libs-only-other)

    # General library
    include_directories(${roscpp_INCLUDE_DIRS})
    add_definitions(${roscpp_CFLAGS})
  else(roscpp_PACKAGE_PATH)
    message(SEND_ERROR "Failed to locate roscpp ROS package")
  endif(roscpp_PACKAGE_PATH)
endif(ROS_CFG)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/trex/ros)

add_library(TREXros SHARED
  # source 
  ros_reactor.cc 
  ros_clock.cc
  ros_timeline.cc
  ros_client.cc
  # header
  ros_reactor.hh
  ros_client.hh
  trex/ros/ros_clock.hh
  trex/ros/ros_subscriber.hh
  trex/ros/bits/ros_timeline.hh)

source_group("Header\ Files\\bits" FILES 
  trex/ros/bits/ros_timeline.hh)

install(DIRECTORY trex/ DESTINATION include/trex/
  FILES_MATCHING PATTERN "*.hh" PATTERN "*.tcc"
  PATTERN "private" EXCLUDE
  PATTERN ".svn" EXCLUDE)
  
target_link_libraries(TREXros ${roscpp_LINK_FLAGS}
  -L${roscpp_LINK_PATH} 
  ${roscpp_LINK_LIBS} TREXagent)
install(TARGETS TREXros DESTINATION lib)
trex_lib(TREXros extra)



# The plugin placeholder : also initialize ros on load
trex_plugin(ros ros_init.cc)
target_link_libraries(ros_pg TREXros)

