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

option(MBARI_TRACKER "Enable ODSS platform tracker." ON)
mark_as_advanced(MBARI_TRACKER) 

if(MBARI_TRACKER)
  find_package(Protobuf)
  check_library_exists(rabbitmq amqp_new_connection "" LIB_AMQP)
  
  if(LIB_AMQP)
    if(PROTOBUF_FOUND)
      include_directories(${PROTOBUF_INCLUDE_DIRS})
      include_directories(${CMAKE_CURRENT_BINARY_DIR})
      protobuf_generate_cpp(TRACKER_PROTO_SRCS TRACKER_PROTO_HDRS platformMessage.proto)
      protobuf_generate_cpp(SENSOR_PROTO_SRCS SENSOR_PROTO_HDRS sensor.proto)
      protobuf_generate_cpp(TREX_PROTO_SRCS TREX_PROTO_HDRS trex.proto)

      add_library(MBARIamqp SHARED
	AMQP_connection.cc
	AMQP_listener.cc
	AMQP_queue.cc
	${TRACKER_PROTO_SRCS}
	${SENSOR_PROTO_SRCS}
	${TREX_PROTO_SRCS}
	DrifterTracker.cc
	location.cc
	MessageHandler.cc
	PositionHandler.cc
	DoradoHandler.cc
	#headers
	AMQP_connection.hh
	AMQP_listener.hh
	AMQP_queue.hh
	DrifterTracker.hh
	point.hh
	location.hh
	serie.hh
	MessageHandler.hh
	PositionHandler.hh
	DoradoHandler.hh
	)
      target_link_libraries(MBARIamqp 
	MBARIshared
	TREXtransaction 
	rabbitmq
	${Boost_LIBRARIES}
	${PROTOBUF_LIBRARIES})
      install(TARGETS MBARIamqp DESTINATION lib)    
      trex_lib(MBARIamqp)

      trex_plugin(dta tracker_plugin.cc)
      target_link_libraries(dta_pg MBARIamqp)          
    else(PROTOBUF_FOUND)
      message(SEND_ERROR "Cannot compile MBARI tracker without protobuf")
    endif(PROTOBUF_FOUND)   
  else(LIB_AMQP)
    message(SEND_ERROR "Cannot compile MBARI tracker without rabbitmq")
  endif(LIB_AMQP)
endif(MBARI_TRACKER)

# install(TARGETS TREXtracker DESTINATION shared/trex/plugins OPTIONAL)
