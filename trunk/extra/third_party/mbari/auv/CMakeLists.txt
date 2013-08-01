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

option(WITH_DORADO "Enable dorado control" OFF)

if(WITH_DORADO) 
  
  if(NOT DORADO_HOME)
    # attempt to find a variable that describes DORADO_HOME
    # is $AUV defined
    set(auv_var $ENV{AUV})
    
    if(NOT EXISTS ${auv_var})
      set(auv_home $ENV{AUV_HOME})
      if(EXISTS ${auv_home})
	set(auv_var ${auv_home}/altex/onboard)
      endif(EXISTS ${auv_home})
    endif(NOT EXISTS ${auv_var})
    
    set(DORADO_HOME ${auv_var} CACHE PATH
      "AUV home directory" FORCE)
  endif(NOT DORADO_HOME)

  if(NOT EXISTS ${DORADO_HOME}) 
    set(DORADO_HOME DORADO_HOME-NOT_FOUND CACHE PATH 
      "Failed to find dorado source code" FORCE)
    message(SEND_ERROR "Failed to find dordado source code.")
  endif(NOT EXISTS ${DORADO_HOME})

  if(NOT OLD_DORADO_HOME EQUAL DORADO_HOME)
    set(DORADO_STATE ${DORADO_HOME}/StatePublisher/StatePacket.x CACHE FILE
      "Location of StatePacket structure description file" FORCE)
    mark_as_advanced(DORADO_STATE)


    set(OLD_DORADO_HOME ${DORADO_HOME} CACHE INTERNAL 
      "Former DOARDO_HOME value" FORCE)
  endif(NOT OLD_DORADO_HOME EQUAL DORADO_HOME)

  if(NOT EXISTS ${DORADO_STATE})
    set(DORADO_STATE DORADO_STATE-NOT_FOUND CACHE PATH
      "Failed to find dorado source code" FORCE)
    message(SEND_ERROR "Failed to locate StatePacket.x")
  endif(NOT EXISTS ${DORADO_STATE})

  find_program(RPC_CMD rpcgen)
  if(NOT EXISTS ${RPC_CMD})
    message(SEND_ERROR "Unable to find rpcgen")
  endif(NOT EXISTS ${RPC_CMD})

  # Generate state packet from xdr
  add_custom_command(
    OUTPUT StatePacket_xdr.c StatePacket.h
    COMMAND ${RPC_CMD} -N -C ${DORADO_STATE}
    DEPENDS ${DORADO_STATE})
  
  include_directories(${CMAKE_CURRENT_BINARY_DIR})

  # Library for connecting to dorado
  add_library(MBARIdorado SHARED
    # source
    StatePacket_xdr.c
    DoradoReactor.cc
    DownlinkReactor.cc
    # headers 
    StatePacket.h
    DoradoReactor.hh
    DownlinkReactor.hh)

  target_link_libraries(MBARIdorado TREXtransaction)
  add_dependencies(mbari MBARIdorado)

endif(WITH_DORADO)