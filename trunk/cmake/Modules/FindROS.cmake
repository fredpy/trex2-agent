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

set(ROS_ROOT $ENV{ROS_ROOT} CACHE PATH "root path for ROS")

if(NOT OLD_ROS_ROOT EQUAL ROS_ROOT)
  find_file(ROS_CMAKE rosbuild.cmake HINTS ${ROS_ROOT}/core/rosbuild)
  if(ROS_CMAKE)
    set(ROS_CONFIG ${ROS_CMAKE} CACHE FILE "ROS configuration file" FORCE)
    include(${ROS_CONFIG})
  else(ROS_CMAKE)
    set(ROS_CONFIG ROS_CONFIG-NOTFOUND CACHE FILE "ROS was not found" FORCE)
  endif(ROS_CMAKE)
  message(STATUS "Looking for ROS configuration: ${ROS_CONFIG}")
  set(OLD_ROS_ROOT ${ROS_ROOT} CACHE INTERNAL "Last ROS_ROOT value" FORCE)
endif(NOT OLD_ROS_ROOT EQUAL ROS_ROOT)


macro(FIND_ROS_PKG name)
  rosbuild_find_ros_package(${name})
  message(STATUS "Looking for ros package ${name}: ${${name}_PACKAGE_PATH}")
  set(ROS_${name}_PACKAGE_PATH ${name}_PACKAGE_PATH CACHE DIRECTORY 
    "path for ros package ${name}" FORCE)
  if(${name}_PACKAGE_PATH)
    rosbuild_invoke_rospack(${name} ROS_${name} INCLUDE_DIRS cflags-only-I)
    mark_as_advanced(ROS_${name}_INCLUDE_DIRS)
    rosbuild_invoke_rospack(${name} ROS_${name} CFLAGS cflags-only-other)
    mark_as_advanced(ROS_${name}_CFLAGS)
    rosbuild_invoke_rospack(${name} ROS_${name} LINK_PATH "libs-only-L")
    mark_as_advanced(ROS_${name}_LINK_PATH)
    rosbuild_invoke_rospack(${name} ROS_${name} LINK_LIBS "libs-only-l")
    mark_as_advanced(ROS_${name}_LINK_LIBS)
    rosbuild_invoke_rospack(${name} ROS_${name} LINK_FLAGS cflags-only-other)
    mark_as_advanced(ROS_${name}_LINK_FLAGS)
  endif(${name}_PACKAGE_PATH)
endmacro(FIND_ROS_PKG)
