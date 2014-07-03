# -*- cmake -*- 
#######################################################################
# Software License Agreement (BSD License)                            #
#                                                                     #
#  Copyright (c) 2014, Frederic Py.                                   #
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

option(WITH_CPP11 "Compile with C++11 support" OFF)

# We need at least boost 1.46.1
find_package(Boost 1.46.1 REQUIRED COMPONENTS
  system filesystem thread regex date_time program_options)

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
