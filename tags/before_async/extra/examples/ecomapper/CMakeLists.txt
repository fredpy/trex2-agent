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
  find_package(ROS)
  
  if(ROS_CONFIG)
    find_ros_pkg(roscpp)
    if(ROS_roscpp_PACKAGE_PATH)
      include_directories(${ROS_roscpp_INCLUDE_DIRS})
      add_definitions(${ROS_roscpp_CFLAGS})
      link_directories(${ROS_roscpp_LINK_PATH})
    else(ROS_roscpp_PACKAGE_PATH)
      message(SEND_ERROR "Failed to find roscpp")
    endif(ROS_roscpp_PACKAGE_PATH)
    
    find_ros_pkg(actionlib)
    if(ROS_actionlib_PACKAGE_PATH)
      include_directories(${ROS_actionlib_INCLUDE_DIRS})
      add_definitions(${ROS_actionlib_CFLAGS})
      link_directories(${ROS_actionlib_LINK_PATH})
    else(ROS_actionlib_PACKAGE_PATH)
      message(SEND_ERROR "Failed to find actionlib")
    endif(ROS_actionlib_PACKAGE_PATH)
    
    find_ros_pkg(ecomapper_msgs)
    if(ROS_ecomapper_msgs_PACKAGE_PATH)
      include_directories(${ROS_ecomapper_msgs_INCLUDE_DIRS})
      add_definitions(${ROS_ecomapper_msgs_CFLAGS})
      link_directories(${ROS_ecomapper_msgs_LINK_PATH})
    else(ROS_ecomapper_msgs_PACKAGE_PATH)
      message(SEND_ERROR "Failed to find ecomapper_msgs")
    endif(ROS_ecomapper_msgs_PACKAGE_PATH)

    find_ros_pkg(gps_common)
    if(ROS_gps_common_PACKAGE_PATH)
      include_directories(${ROS_gps_common_INCLUDE_DIRS})
      add_definitions(${ROS_gps_common_CFLAGS})
      link_directories(${ROS_gps_common_LINK_PATH})
    else(ROS_gps_common_PACKAGE_PATH)
      message(SEND_ERROR "Failed to find gps_common")
    endif(ROS_gps_common_PACKAGE_PATH)
  else(ROS_CONFIG)
    message(SEND_ERROR "UNable to compile ecomapper without ROS support")
  endif(ROS_CONFIG)

  include_directories(${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/trex/ros)

  add_library(TREXecomapper SHARED
    # source
    ecomapper_pg.cc
    ros_listener.cc
    ros_commander.cc
    navigator.cc
    # header
    ros_listener.hh
    ros_commander.hh
    navigator.hh
    spline.hh
    )

  source_group("Header\ Files\\bits" FILES
    trex/ros/bits/ros_timeline.hh)

  install(DIRECTORY trex/ DESTINATION include/trex/
    FILES_MATCHING PATTERN "*.hh" PATTERN "*.tcc"
    PATTERN "private" EXCLUDE
    PATTERN ".svn" EXCLUDE)

  target_link_libraries(TREXecomapper ${ROS_roscpp_LINK_FLAGS}
    ${ROS_actionlib_LINK_FLAGS}
    ${ROS_ecomapper_msgs_LINK_FLAGS}
    ${ROS_gps_common_LINK_FLAGS}
    ${ROS_roscpp_LINK_LIBS}
    ${ROS_actionlib_LINK_LIBS}
    ${ROS_ecomapper_msgs_LIBS}
    ${ROS_gps_common_LIBS}
    TREXagent)

  install(TARGETS TREXecomapper DESTINATION lib)
  trex_lib(TREXecomapper extra)

  trex_plugin(ecomapper ecomapper_pg.cc)
  target_link_libraries(ecomapper_pg TREXecomapper)
endif(WITH_ECOMAPPER)
