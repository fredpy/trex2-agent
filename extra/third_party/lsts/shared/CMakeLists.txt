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

trex_lib(lsts)
target_sources(lsts PRIVATE
  DuneClock.cc
  EuropaExtensions.cc
  ImcAdapter.cc
  LstsUtils.cc
  LstsReactor.cc  
  ImcMessenger.cc
  # headers
  DuneClock.hh
  LstsUtils.hh
  LstsReactor.hh
  trex/lsts/EuropaExtensions.hh
  trex/lsts/ImcAdapter.hh
  trex/lsts/ImcMessenger.hh
)
target_include_directories(lsts PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/trex/lsts
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
  ${DUNE_INCLUDE_DIR}
)
target_link_libraries(lsts europa_core agent ${DUNE_CORE_LIB})

install(DIRECTORY trex/ DESTINATION include/trex/
  FILES_MATCHING PATTERN "*.hh" PATTERN "*.tcc"
  PATTERN "private" EXCLUDE
  PATTERN ".svn" EXCLUDE)


# option(WITH_LSTS_TRACK_AMC "compile track_amc cpu tracking program" OFF)

# if(WITH_LSTS_TRACK_AMC) 
#   # Check for clock_gettime
#   include(CheckLibraryExists)
#   check_library_exists(rt clock_gettime "time.h" HAVE_CLOCK_GETTIME)
#   message(STATUS "Checking for clock_gettime in librt: ${HAVE_CLOCK_GETTIME}")
#   if(HAVE_CLOCK_GETTIME)
#     file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/raw_clock.c
# "#include <time.h>

# int main(int argc, char **argv) {
#   struct timespec date;
#   clock_gettime(CLOCK_MONOTONIC_RAW, &date);
#   return 0;
# }")
#    try_compile(HAS_CLOCK_MONOTONIC_RAW
#      ${CMAKE_CURRENT_BINARY_DIR}
#      ${CMAKE_CURRENT_BINARY_DIR}/raw_clock.c
#      CMAKE_FLAGS -DLINK_LIBRARIES:STRING=rt
#      OUTPUT_VARIABLE OUT)
#    if(NOT HAS_CLOCK_MONOTONIC_RAW)
#      file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/raw_clock.c.out
#        ${OUT})
#    else(NOT HAS_CLOCK_MONOTONIC_RAW)
#      set(RAW_CLK_LIB rt)
#    endif(NOT HAS_CLOCK_MONOTONIC_RAW)

#    message(STATUS "Checking for clock_gettime(CLOCK_MONOTONIC_RAW, ...): ${HAS_CLOCK_MONOTONIC_RAW}")
#  else(HAVE_CLOCK_GETTIME)
#    set(HAS_CLOCK_MONOTONIC_RAW FALSE)
#  endif(HAVE_CLOCK_GETTIME)
 
#  configure_file(raw_clock.hh.in ${CMAKE_CURRENT_BINARY_DIR}/raw_clock.hh)
#  include_directories(${CMAKE_CURRENT_BINARY_DIR})
#  add_executable(track_amc track_amc.cc)
#  install(TARGETS track_amc DESTINATION bin)
#  target_link_libraries(track_amc ${SYSTEM_LIBRARIES}
#    ${Boost_SYSTEM_LIBRARY} ${CHRONO_LIB}
#    ${RAW_CLK_LIB} ${DUNE_CORE_LIB})
#  # expand_libs(track_amc)
# endif(WITH_LSTS_TRACK_AMC)

