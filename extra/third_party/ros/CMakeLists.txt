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
option(ROS_NO_CHECK "Include without checking" OFF)
mark_as_advanced(ROS_NO_CHECK)

if(ROS_NO_CHECK)
  message(STATUS "Include ROS in the project with no checking")
  message(WARNING "Including ROS with no checking result on the "
    "project not being compilable")
else(ROS_NO_CHECK)
  find_package(ROS)
  if(ROS_CONFIG)
    find_ros_pkg(roscpp)
    if(ROS_roscpp_PACKAGE_PATH)
      include_directories(${ROS_roscpp_INCLUDE_DIRS})
      add_definitions(${ROS_roscpp_CFLAGS})
    else(ROS_roscpp_PACKAGE_PATH)
      message(SEND_ERROR "Failed to find roscpp")
    endif(ROS_roscpp_PACKAGE_PATH)

    find_ros_pkg(actionlib)
    if(ROS_actionlib_PACKAGE_PATH)
      include_directories(${ROS_actionlib_INCLUDE_DIRS})
      add_definitions(${ROS_actionlib_CFLAGS})
    else(ROS_actionlib_PACKAGE_PATH)
      message(SEND_ERROR "Failed to find actionlib")
    endif(ROS_actionlib_PACKAGE_PATH)
    
  else(ROS_CONFIG)
    message(SEND_ERROR "Failed to loacte ROS configuration file.")
  endif(ROS_CONFIG)
endif(ROS_NO_CHECK)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
  ${PYTHON_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/trex/ros)
link_directories(${ROS_actionlib_LINK_PATH} ${ROS_roscpp_LINK_PATH})

add_library(TREXros SHARED
  # source
  pyros_reactor.cc
  ros_reactor.cc 
  ros_clock.cc
  ros_timeline.cc
  ros_client.cc
  # header
  ros_reactor.hh
  pyros_reactor.hh
  trex/ros/ros_client.hh
  trex/ros/ros_clock.hh
  trex/ros/ros_convert_traits.hh
  trex/ros/ros_error.hh
  trex/ros/ros_subscriber.hh
  trex/ros/ros_action.hh
  trex/ros/bits/ros_timeline.hh)

install(DIRECTORY trex/ DESTINATION include/trex/
  FILES_MATCHING PATTERN "*.hh" PATTERN "*.tcc"
  PATTERN "private" EXCLUDE
  PATTERN ".svn" EXCLUDE)
  
target_link_libraries(TREXros ${ROS_roscpp_LINK_FLAGS} 
  ${ROS_actionlib_LINK_FLAGS} 
  ${ROS_roscpp_LINK_LIBS} 
  ${ROS_actionlib_LINK_LIBS} 
  TREXagent TREXpython)

install(TARGETS TREXros DESTINATION lib)
trex_lib(TREXros extra)



# The plugin placeholder : also initialize ros on load
trex_plugin(ros ros_init.cc)
target_link_libraries(ros_pg TREXros)

