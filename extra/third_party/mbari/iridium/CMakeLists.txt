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
# Check for Poco
  find_path(POCO_INCLUDE_DIR Poco/Poco.h 
    PATHS /usr/local/include 
    /usr/include 
    /opt/local/include
    )
  find_library(POCO_NET_LIBRARY
    NAMES PocoNet
    PATHS /usr/local/lib 
    /usr/lib 
    /opt/local/lib
    )

if(POCO_NET_LIBRARY)
  include_directories(${POCO_INCLUDE_DIR})
  add_library(MBARIiridium SHARED
    SbdMailer.cc
    DTAReactor.cc
    #headers 
    SbdMailer.hh
    DTAReactor.hh
    )
  target_link_libraries(MBARIiridium 
    TREXtransaction
    ${POCO_NET_LIBRARY})

  install(TARGETS MBARIiridium DESTINATION lib)    
  trex_lib(MBARIiridium extra)

  trex_plugin(iridium iridium_pg.cc)
  target_link_libraries(iridium_pg MBARIiridium)


  # A test program
  add_executable(sender_test test/sender_test.cc)
  target_link_libraries(sender_test MBARIiridium)
  add_custom_target(test COMMENT "Simple tests programs")
  add_dependencies(test sender_test)
  
else(POCO_NET_LIBRARY)
  message(SEND_ERROR "Cannot compile MBARI tracker without Poco")
endif(POCO_NET_LIBRARY)
