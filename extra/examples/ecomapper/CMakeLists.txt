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

option(WITH_ECOMAPPER "Enable Ecomapper" ${WITH_ROS})

if(WITH_ECOMAPPER) 
  if(WITH_ROS AND ROS_CFG)
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
    
    rosbuild_find_ros_package(actionlib)
    if(actionlib_PACKAGE_PATH)
      message(STATUS "Looking for actionlib: ${actionlib_PACKAGE_PATH}")
      # gets compilation information
      rosbuild_invoke_rospack(actionlib actionlib INCLUDE_DIRS cflags-only-I)
      rosbuild_invoke_rospack(actionlib actionlib CFLAGS cflags-only-other)
      rosbuild_invoke_rospack(actionlib actionlib LINK_PATH libs-only-L)
      rosbuild_invoke_rospack(actionlib actionlib LINK_LIBS libs-only-l)
      rosbuild_invoke_rospack(actionlib actionlib LINK_FLAGS libs-only-other)

      # General library
      include_directories(${actionlib_INCLUDE_DIRS})
      add_definitions(${actionlib_CFLAGS})
    else(actionlib_PACKAGE_PATH)
      message(SEND_ERROR "Failed to locate actionlib ROS package")
    endif(actionlib_PACKAGE_PATH)
    
    rosbuild_find_ros_package(ecomapper_msgs)
    if(ecomapper_msgs_PACKAGE_PATH)
      message(STATUS "Looking for ecomapper_msgs: ${ecomapper_msgs_PACKAGE_PATH}")
      # gets compilation information
      rosbuild_invoke_rospack(ecomapper_msgs ecomapper_msgs INCLUDE_DIRS cflags-only-I)
      rosbuild_invoke_rospack(ecomapper_msgs ecomapper_msgs CFLAGS cflags-only-other)
      rosbuild_invoke_rospack(ecomapper_msgs ecomapper_msgs LINK_PATH libs-only-L)
      rosbuild_invoke_rospack(ecomapper_msgs ecomapper_msgs LINK_LIBS libs-only-l)
      rosbuild_invoke_rospack(ecomapper_msgs ecomapper_msgs LINK_FLAGS libs-only-other)

      # General library
      include_directories(${ecomapper_msgs_INCLUDE_DIRS})
      add_definitions(${ecomapper_msgs_CFLAGS})
    else(ecomapper_msgs_PACKAGE_PATH)
      message(SEND_ERROR "Failed to locate ecomapper_msgs ROS package")
    endif(ecomapper_msgs_PACKAGE_PATH)
    
    rosbuild_find_ros_package(gps_common)
    if(gps_common_PACKAGE_PATH)
      message(STATUS "Looking for ecomapper_msgs: ${gps_common_PACKAGE_PATH}")
      # gets compilation information
      rosbuild_invoke_rospack(gps_common gps_common INCLUDE_DIRS cflags-only-I)
      rosbuild_invoke_rospack(gps_common gps_common CFLAGS cflags-only-other)
      rosbuild_invoke_rospack(gps_common gps_common LINK_PATH libs-only-L)
      rosbuild_invoke_rospack(gps_common gps_common LINK_LIBS libs-only-l)
      rosbuild_invoke_rospack(gps_common gps_common LINK_FLAGS libs-only-other)

      # General library
      include_directories(${gps_common_INCLUDE_DIRS})
      add_definitions(${gps_common_CFLAGS})
    else(gps_common_PACKAGE_PATH)
      message(SEND_ERROR "Failed to locate ecomapper_msgs ROS package")
    endif(gps_common_PACKAGE_PATH)
  else(WITH_ROS AND ROS_CFG)
    message(SEND_ERROR "UNable to compile ecomapper without ROS support")
  endif(WITH_ROS AND ROS_CFG)
  
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/trex/ros)
  
  add_library(TREXecomapper SHARED
    # source 
    ecomapper_pg.cc
    ros_listener.cc
    navigator.cc
    # header
    ros_listener.hh
    navigator.hh
    spline.hh
    )
  
  source_group("Header\ Files\\bits" FILES 
    trex/ros/bits/ros_timeline.hh)
  
  install(DIRECTORY trex/ DESTINATION include/trex/
    FILES_MATCHING PATTERN "*.hh" PATTERN "*.tcc"
    PATTERN "private" EXCLUDE
    PATTERN ".svn" EXCLUDE)
  
  target_link_libraries(TREXecomapper ${roscpp_LINK_FLAGS} 
    -L${roscpp_LINK_PATH} 
    ${actionlib_LINK_FLAGS} 
    -L${actionlib_LINK_PATH} 
    ${ecomapper_msgs_LINK_FLAGS}
    -L${ecomapper_msgs_LINK_PATH}
    ${gps_common_LINK_FLAGS}
    -L${gps_common_LINK_PATH}
    ${roscpp_LINK_LIBS} 
    ${actionlib_LINK_LIBS}
    ${ecomapper_msgs_LIBS}
    ${gps_common_LIBS} 
    TREXagent)
  
  install(TARGETS TREXecomapper DESTINATION lib)
  trex_lib(TREXecomapper extra)
  
  trex_plugin(ecomapper ecomapper_pg.cc)
  target_link_libraries(ecomapper_pg TREXecomapper)
endif(WITH_ECOMAPPER)