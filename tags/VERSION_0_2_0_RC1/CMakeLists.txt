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

cmake_minimum_required(VERSION 2.8)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/")

########################################################################
# Project definitions                                                  #
########################################################################
project(trex)
set(TREX_MAJOR 0)
set(TREX_MINOR 2)
set(TREX_PATCH 0)

# CPack version numbers for release tarball name.
set(CPACK_PACKAGE_VERSION_MAJOR ${TREX_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${TREX_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${TREX_PATCH})

set(VERSION "${TREX_MAJOR}.${TREX_MINOR}.${TREX_PATCH}")
message(STATUS "VERSION = ${VERSION}")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "TREX agent excutive")
set(CPACK_PACKAGE_VENDOR "TREX2 development team")
set(CPACK_PACKAGE_DESCRIPTION_FILE ${CMAKE_CURRENT_SOURCE_DIR}/README)
set(CPACK_GENERATOR TGZ)
set(
CPACK_SOURCE_PACKAGE_FILE_NAME
"${CMAKE_PROJECT_NAME}-${VERSION}"
CACHE INTERNAL "tarball basename"
)
set(CPACK_SOURCE_GENERATOR TGZ)
# The following components are regex's to match anywhere (unless anchored)
# in absolute path + filename to find files or directories to be excluded
# from source tarball.
set(CPACK_SOURCE_IGNORE_FILES
"~$"
"log/latest/"
"log/[1-9][0-9]*\\\\.[1-9][0-9]*\\\\.[0-9][0-9]*/"
"/\\\\.svn/"
"\\\\.DS_Store$"
)
#message("CPACK_SOURCE_IGNORE_FILES = ${CPACK_SOURCE_IGNORE_FILES}")
include(CPack)

set(Trex_LIBRARY_DIRS "")
set(Trex_CONFIG_DIRS "")

########################################################################
# Java JDK                                                             #
########################################################################
find_package(Java COMPONENTS Development)

########################################################################
# Boost C++ Libraries                                                  #
########################################################################
find_package(Boost 1.38.0 COMPONENTS thread system)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  list(APPEND Trex_LIBRARY_DIRS ${Boost_LIBRARY_DIRS})
else(Boost_FOUND)
  message(FATAL_ERROR "TREX requires Boost C++ libraries")
endif(Boost_FOUND)

########################################################################
# System libraries                                                     #
########################################################################
include(CheckLibraryExists)

check_library_exists(dl dlopen "" LIB_DL)
if(LIB_DL)
  set(SYSTEM_LIBRARIES ${SYSTEM_LIBRARIES} dl)
else(LIB_DL) 
  message(FATAL_ERROR "TREX requires lib dl")  
endif(LIB_DL)

check_library_exists(pthread pthread_self "" LIB_PTHREAD)
if(LIB_PTHREAD)
  set(SYSTEM_LIBRARIES ${SYSTEM_LIBRARIES} pthread)
endif(LIB_PTHREAD)

check_library_exists(rt shm_unlink "" LIB_RT)
if(LIB_RT)
  set(SYSTEM_LIBRARIES ${SYSTEM_LIBRARIES} rt)
endif(LIB_RT)

########################################################################
# Doxygen                                                              #
########################################################################
find_package(Doxygen)
if(DOXYGEN_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
  add_custom_target(doc
    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
endif(DOXYGEN_FOUND)

########################################################################
# Europa                                                               #
########################################################################
find_package(Europa 
  COMPONENTS PlanDatabase ConstraintEngine TemporalNetwork Utils TinyXml 
  NDDL RulesEngine Solvers System)


########################################################################
# Include Directories                                                  #
########################################################################
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  )

########################################################################
# Compiler definitions                                                 #
########################################################################

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/trex/version.hh.in 
  ${CMAKE_CURRENT_BINARY_DIR}/trex/version.hh @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/trex/version.hh DESTINATION include/trex)

########################################################################
# TREX libraries.                                                      #
########################################################################

add_custom_target(core COMMENT "Trex core libraries")
list(APPEND Trex_LIBRARY_DIRS ${CMAKE_CURRENT_BINARY_DIR})
list(APPEND Trex_CONFIG_DIRS ${CMAKE_SOURCE_DIR}/cfg)

# TREXutils
file(GLOB TREX_UTILS_FILES trex/utils/*.cc)
add_library(TREXutils SHARED ${TREX_UTILS_FILES})
add_dependencies(core TREXutils)
target_link_libraries(TREXutils ${SYSTEM_LIBRARIES})
set_property(SOURCE trex/utils/Pdlfcn.cc 
  PROPERTY COMPILE_DEFINITIONS SUFSHARE="${CMAKE_SHARED_LIBRARY_SUFFIX}")
install(DIRECTORY trex/utils DESTINATION include/trex
          FILES_MATCHING PATTERN "*.hh" PATTERN "*.tcc"
	  PATTERN "private" EXCLUDE
	  PATTERN ".svn" EXCLUDE)

# TREXdomain
file(GLOB TREX_DOMAIN_FILES
  RELATIVE ${PROJECT_SOURCE_DIR} trex/domain/*.cc)
add_library(TREXdomain SHARED ${TREX_DOMAIN_FILES})
add_dependencies(core TREXdomain)
target_link_libraries(TREXdomain TREXutils)
install(DIRECTORY trex/domain DESTINATION include/trex
          FILES_MATCHING PATTERN "*.hh" PATTERN "*.tcc"
	  PATTERN "private" EXCLUDE
	  PATTERN ".svn" EXCLUDE)

# TREXtransaction
file(GLOB TREX_TRANSACTION_FILES
  RELATIVE ${PROJECT_SOURCE_DIR} trex/transaction/*.cc)
add_library(TREXtransaction SHARED ${TREX_TRANSACTION_FILES})
add_dependencies(core TREXtransaction)
target_link_libraries(TREXtransaction TREXdomain)
install(DIRECTORY trex/transaction DESTINATION include/trex
          FILES_MATCHING PATTERN "*.hh" PATTERN "*.tcc"
	  PATTERN "private" EXCLUDE
	  PATTERN ".svn" EXCLUDE)

# TREXagent
file(GLOB TREX_AGENT_FILES 
  RELATIVE ${PROJECT_SOURCE_DIR} trex/agent/*.cc)
add_library(TREXagent SHARED ${TREX_AGENT_FILES})
add_dependencies(core TREXagent)
target_link_libraries(TREXagent TREXtransaction)
install(DIRECTORY trex/agent DESTINATION include/trex
          FILES_MATCHING PATTERN "*.hh" PATTERN "*.tcc"
	  PATTERN ".svn" EXCLUDE
	  PATTERN "private" EXCLUDE)

set(TREX_LIBRARIES TREXutils TREXagent TREXdomain TREXtransaction)

install(TARGETS ${TREX_LIBRARIES} DESTINATION lib)
########################################################################
# TREX plugins.                                                        #
########################################################################
add_custom_target(plugins COMMENT "Trex plugins")

macro(trex_pg name folder)
  get_filename_component(basename ${folder} NAME_WE)
  set(${name} "TREX${basename}")
endmacro(trex_pg) 

macro(trex_plugin folder module)
  trex_pg(plugin_name ${folder})
  set(plugin_files "")
  file(GLOB plugin_files
    RELATIVE ${PROJECT_SOURCE_DIR} ${folder}/*.cc)
  add_library(${plugin_name} SHARED ${plugin_files})
  target_link_libraries(${plugin_name} ${module})
  add_dependencies(plugins ${plugin_name})
  install(TARGETS ${plugin_name} DESTINATION shared/trex/plugins OPTIONAL)
  install(DIRECTORY ${folder}/cfg DESTINATION shared/trex OPTIONAL
    FILES_MATCHING PATTERN "*" 
    PATTERN ".svn" EXCLUDE)
  list(APPEND Trex_CONFIG_DIRS ${CMAKE_SOURCE_DIR}/${folder}/cfg)
endmacro(trex_plugin)

########################################################################
# vitre                                                                #
########################################################################
trex_plugin(extra/vitre TREXtransaction)
trex_pg(vitre_pg extra/vitre)
target_link_libraries(${vitre_pg} ${Boost_LIBRARIES})


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

    set(CMAKE_Java_FLAGS "-sourcepath \"${PROJECT_SOURCE_DIR}\"")

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
      RELATIVE ${CMAKE_SOURCE_DIR} org/trex/vitre/*.java)
    add_library(Vitre EXCLUDE_FROM_ALL ${TREX_vitre_SOURCE_FILES})
    make_jar_executable(Vitre "org.trex.vitre.Vitre")
    add_dependencies(Vitre ${vitre_pg})
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/Vitre.jar OPTIONAL DESTINATION shared/trex/java)
  else(Java_Development_FOUND)
    message(STATUS "Will not compile Vitre.jar for lack of java compilation tools")
  endif(Java_Development_FOUND)
endif(${CMAKE_GENERATOR} MATCHES "Xcode")

########################################################################
# lightswitch                                                          #
########################################################################
trex_plugin(extra/lightswitch TREXtransaction)

########################################################################
# europa                                                               #
########################################################################
if(EUROPA_FOUND) 
  include_directories(${EUROPA_INCLUDE_DIRS})
  list(APPEND Trex_LIBRARY_DIRS ${EUROPA_LIBRARY_DIRS})
  trex_plugin(extra/europa TREXtransaction)
  trex_pg(europa_pg extra/europa)
  set_property(TARGET ${europa_pg} PROPERTY COMPILE_DEFINITIONS ${EUROPA_FLAGS})
  set_property(TARGET ${europa_pg} PROPERTY ${EUROPA_LIBRARIES})
  target_link_libraries(${europa_pg} ${EUROPA_LIBRARIES})
endif(EUROPA_FOUND)

########################################################################
# TREX executables.                                                    #
########################################################################
macro(trex_cmd path)
  get_filename_component(basename ${path} NAME_WE)
  string(TOLOWER ${basename} cmd_name)
  add_executable(${cmd_name} ${path})
  target_link_libraries(${cmd_name} TREXagent)
  add_dependencies(core ${cmd_name})
  install(TARGETS ${cmd_name} DESTINATION bin)  
endmacro(trex_cmd)

trex_cmd(trex/cmds/AMC.cc)
trex_cmd(trex/cmds/Sim.cc)

install(DIRECTORY cfg DESTINATION shared/trex 
  FILES_MATCHING PATTERN "*" 
  PATTERN ".svn" EXCLUDE)

# create the default log directory
install(DIRECTORY log DESTINATION shared/trex/log
  DIRECTORY_PERMISSIONS WORLD_READ WORLD_WRITE
  FILES_MATCHING PATTERN "*" EXCLUDE)
  
list(REMOVE_DUPLICATES Trex_LIBRARY_DIRS)
list(REMOVE_DUPLICATES Trex_CONFIG_DIRS)

# prepare list for bash
string(REPLACE ";" ":" TREX_LIBRARY_DIRS "${Trex_LIBRARY_DIRS}")
string(REPLACE ";" ":" TREX_CONFIG_DIRS "${Trex_CONFIG_DIRS}")

# A file just for testing inside the build directory
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/trex_devel.bash.in
  ${CMAKE_CURRENT_BINARY_DIR}/trex_devel.bash @ONLY)

# create a default bash environement file for installed version
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/trex_init.bash.in
  ${CMAKE_CURRENT_BINARY_DIR}/trex_init.bash @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/trex_init.bash DESTINATION shared/trex)
