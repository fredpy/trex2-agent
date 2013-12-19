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

cmake_minimum_required(VERSION 2.8)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/")

########################################################################
# Project definitions                                                  #
########################################################################
project(trex)
set(TREX_MAJOR 0)
set(TREX_MINOR 5)
set(TREX_PATCH 1)
set(TREX_RC 0)

set(VERSION "${TREX_MAJOR}.${TREX_MINOR}.${TREX_PATCH}")

if(TREX_RC GREATER 0)
  set(VERSION "${VERSION}-rc${TREX_RC}")
endif(TREX_RC GREATER 0)

# CPack version numbers for release tarball name.
set(CPACK_PACKAGE_VERSION ${VERSION})

message(STATUS "${PROJECT_NAME} VERSION = ${VERSION}")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "TREX agent executive")
set(CPACK_PACKAGE_VENDOR "TREX2 development team")
set(CPACK_PACKAGE_DESCRIPTION_FILE ${CMAKE_CURRENT_SOURCE_DIR}/README)
set(CPACK_GENERATOR "TGZ" CACHE STRING "Packaging method for binary")
set(
CPACK_SOURCE_PACKAGE_FILE_NAME
"${CMAKE_PROJECT_NAME}-${VERSION}"
CACHE INTERNAL "tarball basename"
)
set(CPACK_SOURCE_GENERATOR "TGZ" CACHE STRING "Packaging method for source")
# The following components are regex's to match anywhere (unless anchored)
# in absolute path + filename to find files or directories to be excluded
# from source tarball.
set(CPACK_SOURCE_IGNORE_FILES
"~$"
"log/latest/"
"log/[1-9][0-9]*\\\\.[0-9][0-9]*\\\\.[0-9][0-9]*/"
"/\\\\.svn/"
"\\\\.DS_Store$"
)



set(TREX_EXTRA_SRC pkg)
set(TREX_FROM_PKG OFF)

if(IS_DIRECTORY ${CMAKE_SOURCE_DIR}/${TREX_EXTRA_SRC})
  # This code is 
  set(TREX_FROM_PKG ON)
  add_custom_target(trex_prep)
else()
  set(TREX_EXTRA_DIR ${CMAKE_BINARY_DIR}/${TREX_EXTRA_SRC}
    CACHE INTERNAL "Directory where extra generated source can be put")
  set(CPACK_SOURCE_INSTALLED_DIRECTORIES
    "${CMAKE_SOURCE_DIR};/;${TREX_EXTRA_DIR};/${TREX_EXTRA_SRC}")
  add_custom_target(trex_prep)
endif()

include(CPack)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  # Need this for config generated files
  ${CMAKE_BINARY_DIR}/trex/utils
  )

########################################################################
# Linux 64 really needs -fPIC                                          #
########################################################################

#if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
#endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")



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
# Boost C++ general requirement
########################################################################

#set(Boost_USE_STATIC_LIBS        ON)
#set(Boost_USE_MULTITHREADED      ON)
#set(Boost_USE_STATIC_RUNTIME    OFF)

option(WITH_CPP11 "Compile with C++11 support" OFF)

# The minimum requirement 
find_package(Boost 1.46.1 REQUIRED COMPONENTS 
  system filesystem thread regex signals date_time program_options)

if(NOT Boost_FOUND) 
  message(ERROR "Unable to find Boost (>=1.46.1) library")
endif(NOT Boost_FOUND)

include_directories(${Boost_INCLUDE_DIRS})
find_package(Boost COMPONENTS chrono)
message(STATUS "Checking for Boost.Chrono: ${Boost_CHRONO_LIBRARY}")


if(Boost_CHRONO_LIBRARY)
  set(CHRONO_LIB ${Boost_CHRONO_LIBRARY} CACHE PATH "Boost chrono library path" FORCE)
else(Boost_CHRONO_LIBRARY)
  set(CHRONO_LIB "" CACHE PATH "Boost chrono not found" FORCE)
  if(NOT WITH_CPP11)
    message(FATAL_ERROR "Boost.Chrono was not found and WITH_CPP11 is false."
      "Try to enable WITH_CPP11 in order to find C++11 chrono implementation.")
  endif(NOT WITH_CPP11)
endif(Boost_CHRONO_LIBRARY)

########################################################################
# C++11 capabilities                                                   #
########################################################################
if(WITH_CPP11) 
  include(${PROJECT_SOURCE_DIR}/cmake/cpp11/cpp11.cmake)

  if(NOT CHRONO_LIB)
    cpp11_feature_detection(CHRONO)
    if(NOT CPP11_HAS_CHRONO)
      message(ERROR "Did not find required chrono support")
    endif(NOT CPP11_HAS_CHRONO)
  endif(NOT CHRONO_LIB)
  # Helpers to refine the code when cpp11 enabled
  cpp11_feature_detection(UNIQUE_PTR)
  cpp11_feature_detection(SHARED_PTR) 
  cpp11_feature_detection(DELETED_FUNCTIONS)
  cpp11_feature_detection(SYSTEM_ERROR)
  #cpp11_feature_detection(THREAD)

  if(CPP11_ENABLED) 
    set(boost_flags -I${Boost_INCLUDE_DIRS})
    # A simple date_time program : will fail to compile if boost 
    # was linked to a non c++11 compatible standard library
    #   this handle Clang that links against different lib*c++ depending 
    #   on c++11 flags
    cpp11_lib_support(Boost_DATE_TIME "#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>

using namespace boost::posix_time\;

int main(int argc, char *argv[]) {
  ptime today(second_clock::local_time())\;
  std::cout<<today<<std::endl\;
  return 0\;
}" ${boost_flags} ${Boost_DATE_TIME_LIBRARY})


    if(NOT ${CPP11_SUPPORT_Boost_DATE_TIME})
      message(FATAL_ERROR " Boost.DATE_TIME library cannot be used with C++11")
    endif(NOT ${CPP11_SUPPORT_Boost_DATE_TIME})
 
    cpp11_lib_support(Boost_PROGRAM_OPTIONS "#include <boost/program_options.hpp>
#include <iostream>

namespace po=boost::program_options\;

namespace {
  po::options_description opt(\"Test:\")\;
}

int main(int argc, char *argv[]) {
  opt.add_options()
  (\"help,h\", \"should print help\")
  (\"val,v\", po::value< std::vector<std::string> >(), \"A set of strings\")
  (\"int,i\", po::value<int>()->implicit_value(0), \"An integer\")\;

  po::variables_map vals\;

  try {
    po::store(po::command_line_parser(argc,argv).options(opt).run(), vals)\;
    po::notify(vals)\;
  } catch(po::error) {
    return 1\;
  }
  return 0\;
}" ${boost_flags} ${Boost_PROGRAM_OPTIONS_LIBRARY})

    # this test if boost/get_pointer defines get_pointer for std::shared_ptr
    if(CPP11_HAS_SHARED_PTR)
      file(WRITE ${CMAKE_BINARY_DIR}/boost_get_pointer_test.cc "#include <memory>
#include <boost/get_pointer.hpp>

int main(int argv, char *argc[]) {
  std::shared_ptr<int> foo = std::make_shared<int>(0);
  return boost::get_pointer(foo)==foo.get()?0:1; 
}
") 
      try_compile(success
	${CMAKE_BINARY_DIR}
	${CMAKE_BINARY_DIR}/boost_get_pointer_test.cc
	COMPILE_DEFINITIONS ${CPP11_COMPILER_SWITCH} ${boost_flags}
	CMAKE_FLAGS 
	  -DCMAKE_EXE_LINKER_FLAGS:STRING=${CPP11_link_flags} 
	  -DLINK_LIBRARIES:STRING=${LIBS}
	OUTPUT_VARIABLE OUT)
      set(CPP11_BOOST_GET_POINTER_STD ${success})
      if(NOT success)
	file(WRITE ${CMAKE_BINARY_DIR}/boost_get_pointer_test.out ${OUT})
      endif(NOT success)
      message(STATUS "Test if boost::get_pointer support std::shared_ptr: ${CPP11_BOOST_GET_POINTER_STD}") 
    endif(CPP11_HAS_SHARED_PTR)
  

    if(NOT ${CPP11_SUPPORT_Boost_PROGRAM_OPTIONS})
      message(FATAL_ERROR " Boost.Program_Options library cannot be used with C++11")
    endif(NOT ${CPP11_SUPPORT_Boost_PROGRAM_OPTIONS})

    add_definitions(${CPP11_COMPILER_SWITCH})
    # SET (CMAKE_SHARED_LINKER_FLAGS ${CMAKE_SHARED_LINKER_FLAGS_INIT} $ENV{LDFLAGS}
    #   CACHE STRING "Flags used by the linker during the creation of dll's.")
    # set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} ${CPP11_LINK_FLAGS})
    # set(CMAKE_LINK_LIBRARY_FLAGS ${CMAKE_LINK_LIBRARY_FLAGS} ${CPP11_LINK_FLAGS})
    # message(STATUS "CPP11_LINK_FLAGS=${CPP11_LINK_FLAGS}")
    # message(STATUS "CMAKE_LINK_LIBRARY_FLAGS=${CMAKE_LINK_LIBRARY_FLAGS}")
    # # The xcode stuff
    # set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++0x")
    # set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
  endif(CPP11_ENABLED)
else(WITH_CPP11)
  # ensure that cpp11 is not enabled
  set(CPP11_ENABLED FALSE)
endif(WITH_CPP11)

message(STATUS "CPP11 enabled: ${CPP11_ENABLED}")
message(STATUS "CHRONO_LIB: ${CHRONO_LIB}")

########################################################################
# Europa                                                               #
########################################################################

option(WITH_EUROPA "Enable Europa plugin." ON) 

if(WITH_EUROPA) 
  find_package(Europa
    COMPONENTS PlanDatabase ConstraintEngine TemporalNetwork Utils TinyXml 
    NDDL RulesEngine Solvers System)
  if(NOT EUROPA_FOUND)
    message(FATAL_ERROR "Failed to find Europa while WITH_EUROPA is ON")
  endif(NOT EUROPA_FOUND)
endif(WITH_EUROPA)

########################################################################
# Doxygen                                                              #
########################################################################
find_package(Doxygen)
if(DOXYGEN_FOUND)
   configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in 
     ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
   set(DOXYGEN_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/doc CACHE STRING 
     "Output directory for doxygen")
   mark_as_advanced(DOXYGEN_OUTPUT)
   add_custom_target(doc
     ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
     WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
     COMMENT "Generating API documentation with Doxygen" VERBATIM
     )
endif(DOXYGEN_FOUND)

########################################################################
# TREX related macros                                                  #
########################################################################
macro(trex_cfg dir dest)
  install(DIRECTORY ${dir} DESTINATION ${dest} OPTIONAL
    FILES_MATCHING PATTERN "*"
    PATTERN ".svn" EXCLUDE)
  set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_CFGS 
    ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
endmacro(trex_cfg)

function(expand_libs target)
  get_property(deps TARGET ${target} PROPERTY LINK_LIBRARIES)
  if(deps)
    foreach(lib ${deps})
      if(TARGET ${lib})
	get_property(d2 TARGET ${lib} PROPERTY LINK_LIBRARIES)
	if(d2)
	  list(REMOVE_ITEM d2 ${deps})
	  target_link_libraries(${target} ${d2})
	endif(d2)
      endif(TARGET ${lib})
    endforeach()
  endif(deps)
endfunction(expand_libs)
  

function(trex_lib target kind)
  # message(STATUS "trex-lib(${target} ${kind})")
  set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_LIBS 
    ${CMAKE_CURRENT_BINARY_DIR})
  get_property(tmp TARGET ${target} PROPERTY LINK_DEPENDS)
  set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_LIBS 
    ${tmp})
  expand_libs(${target})

  if(CPP11_ENABLED) 
    set_target_properties(${target} PROPERTIES LINK_FLAGS ${CPP11_LINK_FLAGS})
  endif(CPP11_ENABLED)

  # message(STATUS "${target}: ${i_incs}")
  # transmit directory include to target include for older cmake
  get_property(tmp DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
  set_property(TARGET ${target} APPEND PROPERTY INCLUDE_DIRECTORIES ${tmp})
  unset(tmp)
  
  if(kind) 
    # update global include path for installed trex

    get_property(incs TARGET ${target} PROPERTY INCLUDE_DIRECTORIES)
    if(incs)
      # cleanup 
      list(REMOVE_DUPLICATES incs)
      set(i_incs ${CMAKE_INSTALL_PREFIX}/include)
      foreach(i ${incs})
	if(NOT ${i} MATCHES ${CMAKE_SOURCE_DIR})
	  if(NOT ${i} MATCHES ${CMAKE_BINARY_DIR})
	    list(APPEND i_incs ${i})
	  endif(NOT ${i} MATCHES ${CMAKE_BINARY_DIR})
	endif(NOT ${i} MATCHES ${CMAKE_SOURCE_DIR})
      endforeach()
      if(i_incs)
	list(REMOVE_DUPLICATES i_incs)
      endif(i_incs)
      unset(incs)
      set_property(GLOBAL APPEND PROPERTY TREX_INCLUDES ${i_incs})
      unset(i_incs)
    endif(incs)

    # message(STATUS "trex-${kind} += ${target}")
    install(TARGETS ${target} DESTINATION lib 
      EXPORT trex-targets)
    set_property(GLOBAL APPEND PROPERTY trex-${kind} ${target})
  endif(kind)
endfunction(trex_lib)

function(trex_py target)
  set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_PYTHON ${CMAKE_CURRENT_BINARY_DIR})
  expand_libs(${target})
  if(CPP11_ENABLED) 
    set_target_properties(${target} PROPERTIES LINK_FLAGS ${CPP11_LINK_FLAGS})
  endif(CPP11_ENABLED)
  # trex_lib(${target} FALSE)
  install(TARGETS ${target} DESTINATION ${TREX_SHARED}/python OPTIONAL EXPORT trex-targets) 
  set_property(GLOBAL APPEND PROPERTY trex-python ${target})
endfunction(trex_py)

macro(trex_cmd target)
  set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_CMDS
    ${CMAKE_CURRENT_BINARY_DIR})
  expand_libs(${target})
  if(CPP11_ENABLED) 
    set_target_properties(${target} PROPERTIES LINK_FLAGS ${CPP11_LINK_FLAGS})
  endif(CPP11_ENABLED)
  get_property(tmp TARGET ${target} PROPERTY LINK_DEPENDS)
  set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_LIBS 
    ${tmp})
  unset(tmp)
endmacro(trex_cmd)

macro(bash_path var property) 
  get_property(tmp GLOBAL PROPERTY ${property})
  if(tmp)
    list(REMOVE_DUPLICATES tmp)
  endif(tmp)
  string(REPLACE ";" ":" ${var} "${tmp}")
  unset(tmp)
endmacro(bash_path)

########################################################################
# TREX sub directories                                                 #
########################################################################

set(TREX_SHARED share/trex)

include_directories(${CMAKE_BINARY_DIR})

# core libraries 
add_subdirectory(trex)
trex_cfg(cfg ${TREX_SHARED})

# include extra/plugins
add_subdirectory(extra) 

########################################################################
# Finalize                                                             #
########################################################################

install(DIRECTORY scripts/ DESTINATION bin OPTIONAL
  FILES_MATCHING PATTERN "*"
  PATTERN "*.in" EXCLUDE
  PATTERN ".svn" EXCLUDE)
set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_CMDS
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts)


bash_path(TREX_CONFIG_DIRS ${PROJECT_NAME}_CFGS)
bash_path(TREX_LIBRARY_DIRS ${PROJECT_NAME}_LIBS)
bash_path(TREX_BINARY_DIRS ${PROJECT_NAME}_CMDS)
bash_path(TREX_PYTHON ${PROJECT_NAME}_PYTHON)

# A file just for testing inside the build directory
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/scripts/trex_devel.bash.in
  ${CMAKE_CURRENT_BINARY_DIR}/trex_devel.bash @ONLY)

# create a default bash environement file for installed version
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/scripts/trex_init.bash.in
  ${CMAKE_CURRENT_BINARY_DIR}/trex_init.bash @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/trex_init.bash DESTINATION ${TREX_SHARED})

configure_file(cmake/trex-config-version.cmake.in 
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/trex-config-version.cmake @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/trex-config-version.cmake 
  DESTINATION share/trex/cmake)



file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/log)
file(WRITE ${CMAKE_BINARY_DIR}/log/.empty_file " ") # Not really empty ... it is just to force tar to use it

# create the default log directory ... everybody can read/write/exec
install(DIRECTORY ${CMAKE_BINARY_DIR}/log DESTINATION ${TREX_SHARED}
   DIRECTORY_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE 
    GROUP_READ GROUP_WRITE GROUP_EXECUTE
    WORLD_READ WORLD_WRITE WORLD_EXECUTE
   FILES_MATCHING PATTERN "latest" EXCLUDE
   PATTERN "????.???.*" EXCLUDE
   PATTERN "archives" EXCLUDE
   PATTERN ".empty_file")

get_property(TREX_INCS GLOBAL PROPERTY TREX_INCLUDES)
get_property(trex-core GLOBAL PROPERTY trex-core)
get_property(trex-extra GLOBAL PROPERTY trex-extra)
get_property(trex-python GLOBAL PROPERTY trex-python)
list(REMOVE_DUPLICATES TREX_INCS)
if(CPP11_ENABLED)
  set(trex_FLAGS ${CPP11_COMPILER_SWITCH})
endif(CPP11_ENABLED)
configure_file(cmake/trex-config.cmake.in 
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/trex-config.cmake @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/trex-config.cmake 
  DESTINATION share/trex/cmake)
unset(TREX_INCS)

# All the packages are mixed together so older version of cmake are not confused
install(EXPORT trex-targets DESTINATION share/trex/cmake/imports OPTIONAL)

#include(CMakePackageConfigHelpers)
# Note I need to do a real version checking
#write_basic_package_version_file(trexConfigVersion.cmake VERSION ${VERSION} COMPATIBILITY AnyNewerVersion)


