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

option(WITH_VITRE "Enable Vitre plugin." ON) 

if(WITH_VITRE)
  find_package(Java COMPONENTS Development)

  trex_plugin(vitre Vitre.cc)
  target_link_libraries(vitre_pg TREXtransaction ${Boost_LIBRARIES})

  if(${CMAKE_GENERATOR} MATCHES "Xcode")
    message(STATUS "Will not compile Vitre.jar under Xcode")
  else(${CMAKE_GENERATOR} MATCHES "Xcode")
    if(Java_Development_FOUND)
      enable_language(Java)
      
      # a macro to indicate to java where is the main class
      macro(make_jar_executable target main_class)
	get_target_property(var_loc "${target}" LOCATION)
	add_custom_command(TARGET ${target} POST_BUILD 
	  COMMAND ${CMAKE_Java_ARCHIVE} ufe ${var_loc} ${main_class} 
	  COMMENT "Creating JAR entry point.")
	UNSET(var_loc)
      endmacro()

      set(CMAKE_Java_FLAGS "-sourcepath \"${CMAKE_CURRENT_SOURCE_DIR}\"")

      # following is a patch/hack to make cmake compiles jars files properly
      # compile a Java file into an object file
      file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/.class")
      set(CMAKE_Java_COMPILE_OBJECT
	"<CMAKE_Java_COMPILER> <FLAGS> <SOURCE> -d \"<CMAKE_CURRENT_BINARY_DIR>/.class\""
	)
      # this is a place holder if java needed flags for javac they would go here.
      set(CMAKE_Java_CREATE_STATIC_LIBRARY
	"<CMAKE_Java_ARCHIVE> -cf <TARGET> -C \"<CMAKE_CURRENT_BINARY_DIR>/.class\" ."
	)
      # end of the patch 
      file(GLOB_RECURSE TREX_vitre_SOURCE_FILES
	RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} org/trex/vitre/*.java)

      add_library(Vitre EXCLUDE_FROM_ALL ${TREX_vitre_SOURCE_FILES})

      make_jar_executable(Vitre "org.trex.vitre.Vitre")
      add_dependencies(Vitre vitre_pg)
      install(FILES ${CMAKE_CURRENT_BINARY_DIR}/Vitre.jar OPTIONAL 
	DESTINATION shared/trex/java)
      
    else(Java_Development_FOUND)
      message(STATUS "Will not compile Vitre.jar for lack of java compilation tools")
    endif(Java_Development_FOUND)
  endif(${CMAKE_GENERATOR} MATCHES "Xcode")
endif(WITH_VITRE)