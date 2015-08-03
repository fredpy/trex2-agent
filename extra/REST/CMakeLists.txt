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

option(WITH_REST "Enable REST based web API for trex agent" OFF)

if(WITH_REST) 
  if(WITH_WT_SERVER) 
    if(Wt_DBOSQLITE3_LIBRARY)
      set(DBO_BACKEND Sqlite3)
      set(DBO_EXTENSION ".db3")
      set(DBO_LIBS ${Wt_DBO_LIBRARY} ${Wt_DBOSQLITE3_LIBRARY})
      set(DBO_FOUND TRUE)
    endif(Wt_DBOSQLITE3_LIBRARY)

    if(DBO_FOUND) 
      configure_file(${CMAKE_CURRENT_SOURCE_DIR}/dbo_cfg.hh.in
	${CMAKE_CURRENT_BINARY_DIR}/dbo_cfg.hh @ONLY)
      include_directories(${TREX_WT_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR})
      trex_plugin(REST
	# source
	db_manager.cc
	tick_manager.cc
	timeline_services.cc
	REST_pg.cc
	REST_reactor.cc
	REST_service.cc
	TimelineHistory.cc
	# headers
	db_manager.hh
	tick_manager.hh
	timeline_services.hh
	timeline_wrap.hh
	REST_reactor.hh
	REST_service.hh
	TimelineHistory.hh)

      target_link_libraries(REST_pg
	TREXwt_server ${DBO_LIBS}
	TREXtransaction)

      set(REST_HOST "localhost" CACHE STRING "default REST server address.")
      set(REST_PORT 8080 CACHE NUMBER "default rest server port.")
      
      configure_file(${CMAKE_CURRENT_SOURCE_DIR}/rest.xml.in
	${CMAKE_CURRENT_BINARY_DIR}/cfg/rest.xml @ONLY)
      install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cfg/rest.xml 
	DESTINATION ${TREX_SHARED}/cfg)
      set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_CFGS 
	${CMAKE_CURRENT_BINARY_DIR}/cfg)
    else(DBO_FOUND)
      message(WARNING "Unable to find Wt/Dbo!!! Disabling REST plugin")
      set(WITH_REST OFF CACHE BOOL "Disabled for the lack of Wt/Dbo" FORCE)
    endif(DBO_FOUND)
  else(WITH_WT_SERVER)
    message(SEND_ERROR "Cannot enable WITH_REST plugin while WITH_WT_SERVER is OFF")
  endif(WITH_WT_SERVER)      
endif(WITH_REST)