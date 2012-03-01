# -*- cmake -*-
########################################################################
# Copyright (c) 2011, Ricardo Martins                                  #
# All rights reserved.                                                 #
#                                                                      #
# Redistribution and use in source and binary forms, with or without   #
# modification, are permitted provided that the following conditions   #
# are met:                                                             #
#                                                                      #
# Redistributions of source code must retain the above copyright       #
# notice, this list of conditions and the following disclaimer.        #
# Redistributions in binary form must reproduce the above copyright    #
# notice, this list of conditions and the following disclaimer in the  #
# documentation and/or other materials provided with the distribution. #
#                                                                      #
# Neither the name of the Universidade do Porto - Faculdade de         #
# Engenharia nor the names of its contributors may be used to endorse  #
# or promote products derived from this software without specific      #
# prior written permission.                                            #
#                                                                      #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS  #
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT    #
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS    #
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE       #
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, #
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, #
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;     #
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER     #
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT   # 
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN    #
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE      #
# POSSIBILITY OF SUCH DAMAGE.                                          #
########################################################################

option(WITH_EUROPA "Enable Europa plugin." ON) 


if(WITH_EUROPA) 
  find_package(Europa 
    COMPONENTS PlanDatabase ConstraintEngine TemporalNetwork Utils TinyXml 
    NDDL RulesEngine Solvers System)
  if(EUROPA_FOUND)

    configure_file(bits/europa_config.hh.in 
      ${CMAKE_CURRENT_BINARY_DIR}/bits/europa_config.hh)
    install(
      FILES ${CMAKE_CURRENT_BINARY_DIR}/bits/europa_config.hh 
      DESTINATION include/trex/europa/bits
      )

    include_directories(${EUROPA_INCLUDE_DIRS})
    include_directories(${CMAKE_CURRENT_BINARY_DIR})

    add_subdirectory(core)

    add_library(TREXeuropa SHARED 
      EuropaReactor.cc
      europa_convert.cc
      EuropaDomain.cc  
      extensions/EarliestFirstFlawManager.cc      
      # headers
      config.hh
      EuropaReactor.hh
      EuropaEntity.hh
      bits/europa_convert.hh
      extensions/EarliestFirstFlawManager.hh
    )

    target_link_libraries(TREXeuropa TREXeuropa_core TREXtransaction)
  
    install(TARGETS TREXeuropa DESTINATION lib) 
    trex_lib(TREXeuropa)
  
    install(DIRECTORY . DESTINATION include/trex/europa
      FILES_MATCHING PATTERN "*.hh" PATTERN "*.tcc"
      PATTERN "private" EXCLUDE
      PATTERN "deprecated" EXCLUDE
      PATTERN "cfg" EXCLUDE
      PATTERN ".svn" EXCLUDE)
  
    trex_plugin(europa europa_plugin.cc)
    target_link_libraries(europa_pg TREXeuropa)
  else(EUROPA_FOUND)
    message(FATAL_ERROR "Failed to find Europa while WITH_EUROPA is ON")
  endif(EUROPA_FOUND)
endif(WITH_EUROPA)
