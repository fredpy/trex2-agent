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

option(WITH_MBARI "Enable MBARI specific code." OFF) 

if(WITH_MBARI)
  add_custom_target(mbari COMMENT "MBARI specific targets")
  trex_cfg(cfg ${TREX_SHARED})

  add_subdirectory(shared)
  add_subdirectory(tracker)
  add_subdirectory(iridium)
  add_subdirectory(europa)

  #set(POCO_HINT "" CACHE FILEPATH "Hint to find Poco libraries")
  #find_library(POCO_NET_LIBRARIES
  #  NAMES PocoNet 
  #  HINTS ${POCO_HINT}
  #  DOCS "Looking for Poco library")
  #find_path(POCO_INCLUDE "Poco/Poco.h" HINTS ${POCO_HINT})
  #if(POCO_NET_LIBRARIES AND POCO_INCLUDE)
  #  set(POCO_FOUND ON)
  #else(POCO_NET_LIBRARIES AND POCO_INCLUDE)
  #  message(WARNING "Unable to find Poco libraries: disabling iridium plugin")
  #  set(POCO_FOUND OFF)
  #endif(POCO_NET_LIBRARIES AND POCO_INCLUDE)
endif(WITH_MBARI)

