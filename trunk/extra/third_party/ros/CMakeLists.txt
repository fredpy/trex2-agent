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

# Try to locate ROS 

set(ROS_ROOT $ENV{ROS_ROOT} CACHE PATH
  "Location of ros in the system")


# Search for ROS
if(NOT OLD_ROS_ROOT EQUAL ROS_ROOT)
  find_file(ros_cmake rosbuild.cmake HINTS $ROS_ROOT/core/rosbuild)
  if(ros_cmake) 
    set(ROS_BUILD ${ros_cmake} CACHE FILE "ROS cmake build file")
  else(ros_cmake)
    message(SEND_ERROR "Unable to find rosbuild.cmake")
  endif(ros_cmake)
  set(OLD_ROS_ROOT ${ROS_ROOT} CACHE INTERNAL "former ROS_ROOT value." FORCE)
endif(NOT OLD_ROS_ROOT EQUAL ROS_ROOT)

if(ROS_BUILD)
  include($ROS_BUILD)
  rosbuild_init()
  
  rosbuild_add_library(TREXros ros_reactor.cc ros_reactor.hh)
  target_link_libraries(TREXros TREXtransaction)
  
  trex_plugin(ros ros_plugin.cc)
  target_link_libraries(ros_pg TREXros)
endif(ROS_BUILD)