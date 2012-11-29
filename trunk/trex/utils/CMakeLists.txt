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

configure_file(platform/bits/cpp11.hh.in 
  ${CMAKE_CURRENT_BINARY_DIR}/bits/cpp11.hh)

install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/bits/cpp11.hh 
  DESTINATION include/trex/utils/platform/bits
)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(TREXutils SHARED
  # real source
  ErrnoExcept.cc
  Exception.cc
  LogManager.cc
  Pdlfcn.cc
  PluginLoader.cc
  SingletonDummy.cc
  SingletonServer.cc
  TREXversion.cc
  XmlUtils.cc
  asio_runner.cc
  log/entry.cc
  log/text_log.cc
  # headers
  chrono_helper.hh
  ErrnoExcept.hh
  Exception.hh
  Factory.hh
  Hashable.hh
  id_mapper.hh
  IOstreamable.hh
  LogManager.hh
  Plugin.hh
  PluginLoader.hh
  SharedVar.hh
  SingletonUse.hh
  bits/SingletonDummy.hh
  bits/SingletonServer_fwd.hh
  bits/SingletonWrapper.hh
  StringExtract.hh
  Symbol.hh
  tick_clock.hh
  TimeUtils.hh
  TREXversion.hh
  XmlFactory.hh
  XmlUtils.hh
  asio_runner.hh
  log/log_fwd.hh
  log/entry.hh
  log/stream.hh
  log/bits/log_stream.hh
  log/bits/log_sig.hh
  log/text_log.hh
  log/out_file.hh
  platform/chrono.hh 
  platform/memory.hh
  # template source code
  bits/Factory.tcc
  bits/SingletonUse.tcc
  bits/SingletonWrapper.tcc
  bits/Symbol.tcc
  bits/XmlFactory.tcc
)

# map ditectory structure for XCode 
source_group("Source\ Files\\log" FILES 
  log/entry.cc
  log/text_log.cc
)

source_group("Header\ Files\\bits" FILES 
  bits/SingletonDummy.hh
  bits/SingletonServer_fwd.hh
  bits/SingletonWrapper.hh
)

source_group("Header\ Files\\platform" FILES
  platform/chrono.hh 
  platform/memory.hh
  platform/cpp11_deleted.hh
)

source_group("Header\ Files\\log" FILES 
  log/log_fwd.hh
  log/entry.hh
  log/stream.hh
  log/text_log.hh
  log/out_file.hh
)

source_group("Header\ Files\\log\\bits" FILES 
  log/bits/log_stream.hh
  log/bits/log_sig.hh
)

source_group("Template Files\\bits" FILES  
  bits/Factory.tcc
  bits/SingletonUse.tcc
  bits/SingletonWrapper.tcc
  bits/Symbol.tcc
  bits/XmlFactory.tcc
)

add_dependencies(core TREXutils)
target_link_libraries(TREXutils ${SYSTEM_LIBRARIES}  
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
  ${CHRONO_LIB}
  ${Boost_DATE_TIME_LIBRARY}
  ${Boost_THREAD_LIBRARY})
trex_lib(TREXutils core)

set_property(SOURCE Pdlfcn.cc 
  PROPERTY COMPILE_DEFINITIONS SUFSHARE="${CMAKE_SHARED_LIBRARY_SUFFIX}")

install(DIRECTORY . DESTINATION include/trex/utils
  FILES_MATCHING PATTERN "*.hh" PATTERN "*.tcc"
  PATTERN "private" EXCLUDE
  PATTERN ".svn" EXCLUDE)
