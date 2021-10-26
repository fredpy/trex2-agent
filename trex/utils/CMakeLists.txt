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

option(ASIO_DEBUG "enable debug of asio" OFF)
mark_as_advanced(ASIO_DEBUG)
configure_file(bits/asio_conf.hh.in
  ${CMAKE_CURRENT_BINARY_DIR}/bits/asio_conf.hh)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/bits/asio_conf.hh
  DESTINATION include/trex/utils/bits
)
include(git_version)
git_header(bits/git_version.hh)
set_source_files_properties(TREXversion.cc
  PROPERTIES OBJECT_DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/bits/git_version.hh)

trex_lib(utils)
target_sources(utils PRIVATE
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
  ptree_io.cc
  asio_runner.cc
  asio_fstream.cc
  priority_strand.cc
  private/priority_strand_impl.cc
  log/entry.cc
  log/text_log.cc
  cpu_clock.cc
  # headers
  ${CMAKE_CURRENT_BINARY_DIR}/bits/git_version.hh
  asio_fstream.hh
  asio_signal.hh
  asio_signal_n.hh
  asio_signal_fwd.hh
  bits/asio_signal_base.hh
  bits/asio_signal_iter.hh
  bits/asio_signal_template.hh
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
  priority_strand.hh
  private/priority_strand_impl.hh
  SharedVar.hh
  SingletonUse.hh
  bits/async_result.hh
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
  ptree_io.hh
  asio_runner.hh
  cpu_clock.hh
  log/log_fwd.hh
  log/entry.hh
  log/stream.hh
  log/bits/log_stream.hh
  log/bits/log_sig.hh
  log/text_log.hh
  log/out_file.hh
  log/log_pipe.hh
  ${CMAKE_CURRENT_BINARY_DIR}/bits/asio_conf.hh
  # template source code
  bits/Factory.tcc
  bits/priority_strand.tcc
  bits/SingletonUse.tcc
  bits/SingletonWrapper.tcc
  bits/Symbol.tcc
  bits/XmlFactory.tcc
)
target_compile_features(utils PUBLIC cxx_std_17)
target_include_directories(utils PRIVATE 
 ${CMAKE_CURRENT_BINARY_DIR}
 ${CMAKE_BINARY_DIR}
PUBLIC
 $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
 $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
 $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
 $<INSTALL_INTERFACE:include>
)
target_link_libraries(utils dl Boost::system Boost::date_time Boost::thread)

set_property(SOURCE Pdlfcn.cc 
  PROPERTY COMPILE_DEFINITIONS SUFSHARE="${CMAKE_SHARED_LIBRARY_SUFFIX}")

install(DIRECTORY . DESTINATION include/trex/utils
  FILES_MATCHING PATTERN "*.hh" PATTERN "*.tcc"
  PATTERN "private" EXCLUDE)
