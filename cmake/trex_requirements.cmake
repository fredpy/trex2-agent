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

set(LINK_AS_NEEDED)
set(LINK_NO_AS_NEEDED)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(LINK_AS_NEEDED "-Wl,--as-needed")
  set(LINK_NO_AS_NEEDED "-Wl,--no-as-needed")
endif()

########################################################################
# System libraries                                                     #
########################################################################
include(CheckLibraryExists)

check_library_exists(dl dlopen "" HAVE_DL)
if(NOT HAVE_DL)  
  message(FATAL_ERROR "TREX requires lib dl")  
endif(NOT HAVE_DL)

find_package(Boost 1.70 REQUIRED COMPONENTS
  system regex date_time program_options thread)


if(NOT Boost_FOUND) 
  message(ERROR "Unable to find Boost (>=1.46.1) library")
endif(NOT Boost_FOUND)

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
