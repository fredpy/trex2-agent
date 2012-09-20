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

option(WITH_WITRE "Enable witre plugin (require Wt)" ON)

if(WITH_WITRE) 
  find_package(Wt)

  if(Wt_FOUND) 
    include_directories(${Wt_INCLUDE_DIR})
    add_library(TREXwitre SHARED
       Popup.cc 
       Witre.cc 
       WitreServer.cc 
       # headers
       Popup.hh
       Witre.hh
       WitreGraph.hh
       WitreServer.hh
     )
   target_link_libraries(TREXwitre 
     ${Wt_LIBRARY} ${Wt_HTTP_LIBRARY}
     ${Boost_THREAD_LIBRARY} 
     ${Boost_SIGNALS_LIBRARY} 
     TREXtransaction
   ) 
   install(TARGETS TREXwitre DESTINATION lib)

   trex_plugin(witre witre_pg.cc)
   target_link_libraries(witre_pg TREXwitre)

    # generate the witre.xml configuration file with a doocroot pointing to witre
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/witre.xml.in
      ${CMAKE_CURRENT_BINARY_DIR}/cfg/witre.xml @ONLY)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cfg/witre.xml DESTINATION ${TREX_SHARED}/cfg)
    set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_CFGS 
      ${CMAKE_CURRENT_BINARY_DIR}/cfg)
  else(Wt_FOUND)
    message(WARNING "Unable to find Wt!!! Disabling witre plugin")
    set(WITH_WITRE OFF CACHE BOOL "Disabled for the lack of Wt" FORCE)
  endif(Wt_FOUND)
endif(WITH_WITRE)
