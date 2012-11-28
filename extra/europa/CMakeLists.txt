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



if(WITH_EUROPA) 

  if(EUROPA_HAVE_EFFECT)
    # generate lightswitch.nddl for europa 2.6
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cfg/lightswitch_26.nddl
      ${CMAKE_CURRENT_BINARY_DIR}/cfg/lightswitch.nddl COPY_ONLY)
  else(EUROPA_HAVE_EFFECT)
    message(WARNING "Your europa version appears to be older than 2.6."
      "Some new trex features have been disabled.")
    # generate lightswitch.nddl for europa 2.5
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cfg/lightswitch_25.nddl
      ${CMAKE_CURRENT_BINARY_DIR}/cfg/lightswitch.nddl COPY_ONLY)    
  endif(EUROPA_HAVE_EFFECT)

  # install lightswitch.nddl
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cfg/lightswitch.nddl
    DESTINATION ${TREX_SHARED}/cfg)
  set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_CFGS 
    ${CMAKE_CURRENT_BINARY_DIR}/cfg)
  
  # plan archival options
  
  set(EUROPA_ARCHIVALValues "EuropaDefault;Greedy")

  set(EUROPA_ARCHIVAL "Greedy" CACHE STRING
    "Europa archival technique used (${EUROPA_ARCHIVALValues})")


  set_property(CACHE EUROPA_ARCHIVAL 
    PROPERTY STRINGS ${EUROPA_ARCHIVALValues})

  mark_as_advanced(EUROPA_ARCHIVAL)

  # Check if valid option
  list(FIND EUROPA_ARCHIVALValues ${EUROPA_ARCHIVAL} 
    EUROPA_ARCHIVAL_VALID)

  if(EUROPA_ARCHIVAL_VALID LESS 0) 
    message(ERROR "EUROPA_ARCHIVAL='${EUROPA_ARCHIVAL}' is not a valid option.\n"
      "Please select one of : ${EUROPA_ARCHIVALValues}")
  else(EUROPA_ARCHIVAL_VALID LESS 0)
    message(STATUS "EUROPA_ARCHIVAL='${EUROPA_ARCHIVAL}'")
  endif(EUROPA_ARCHIVAL_VALID LESS 0)

  # put the europa flags under trex/europa/bits
  configure_file(bits/europa_config.hh.in 
    ${CMAKE_BINARY_DIR}/bits/europa_config.hh)
  install(
    FILES ${CMAKE_BINARY_DIR}/bits/europa_config.hh 
    DESTINATION include/trex/europa/bits
    )
  

  
  include_directories(${CMAKE_CURRENT_SOURCE_DIR})
  include_directories(${EUROPA_INCLUDE_DIRS})
  include_directories(${CMAKE_BINARY_DIR})

  add_subdirectory(core)

  add_library(TREXeuropa SHARED 
    EuropaReactor.cc
    europa_convert.cc
    EuropaDomain.cc  
    extensions/Bind.cc
    extensions/DoNotMatchFilter.cc
    extensions/extensions.cc
    extensions/EarliestFirstFlawManager.cc
    extensions/GoalFilter.cc
    extensions/Numeric.cc
    extensions/Trigonometry.cc
    # headers
    EuropaReactor.hh
    bits/europa_convert.hh
    EuropaEntity.hh
    extensions/Bind.hh
    extensions/DoNotMatchFilter.hh
    extensions/EarliestFirstFlawManager.hh
    extensions/GoalFilter.hh
    extensions/Numeric.hh
    extensions/Trigonometry.hh
  )

  target_link_libraries(TREXeuropa TREXeuropa_core TREXtransaction)
  
  install(TARGETS TREXeuropa DESTINATION lib) 
  trex_lib(TREXeuropa extra)
  
  trex_plugin(europa europa_plugin.cc)
  target_link_libraries(europa_pg TREXeuropa)

  install(DIRECTORY trex/ DESTINATION include/trex/
    FILES_MATCHING PATTERN "*.hh" PATTERN "*.tcc"
    PATTERN "private" EXCLUDE
    PATTERN ".svn" EXCLUDE)

endif(WITH_EUROPA)
