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

# Check for dynamic load functions

configure_file(generated/dl_info.hh.in
  ${CMAKE_CURRENT_BINARY_DIR}/private/dl_info.hh)
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

add_library(TREXutils SHARED
  # real source
  factory_error.cc
  log_manager.cc
  private/log_mgmt_impl.cc
  private/platform/${DL_TYPE}/dl_func.cc
  plugin_loader.cc
  trex_version.cc
  xml_utils.cc
  ptree_io.cc
  priority_strand.cc
  private/priority_strand_impl.cc
  asio_runner.cc
  asio_fstream.cc
  private/async_ofstream_impl.cc
  log/entry.cc
  log/text_log.cc
  singleton/dummy.cc
  singleton/server.cc
  timing/cpu_clock.cc
  timing/posix_utils.cc
  # headers
  priority_strand.hh
  private/priority_strand_impl.hh
  asio_fstream.hh
  private/async_ofstream_impl.hh
  asio_signal.hh
  asio_signal_fwd.hh
  bits/asio_signal_n.hh
  bits/stranded_service.hh
  bits/async_result.hh
  bits/asio_signal_base.hh
  bits/asio_signal_iter.hh
  bits/asio_signal_template.hh
  factory_error.hh
  generic_factory.hh
  id_mapper.hh
  log_manager.hh
  private/log_mgmt_impl.hh
  Plugin.hh
  plugin_loader.hh
  private/dl_func.hh
  ${CMAKE_CURRENT_BINARY_DIR}/private/dl_info.hh
  private/platform/${DL_TYPE}/dl_func.hh
  shared_var.hh
  symbol.hh
  basic_symbol.hh
  trex_version.hh
  xml_factory.hh
  xml_utils.hh
  ptree_io.hh
  asio_runner.hh
  log/log_fwd.hh
  log/entry.hh
  log/stream.hh
  log/bits/log_stream.hh
  log/bits/log_sig.hh
  log/text_log.hh
  log/out_file.hh
  log/log_pipe.hh
  singleton.hh
  singleton/use.hh
  singleton/bits/dummy.hh
  singleton/bits/server_fwd.hh
  singleton/bits/wrapper.hh
  singleton/private/server.hh
  timing/chrono_helper.hh
  timing/cpu_clock.hh
  timing/tick_clock.hh
  timing/posix_utils.hh
  # template source code
  bits/priority_strand.tcc
  bits/generic_factory.tcc
  singleton/bits/use.tcc
  singleton/bits/wrapper.tcc
  bits/basic_symbol.tcc
  bits/xml_factory.tcc
)

add_dependencies(core TREXutils)
add_dependencies(TREXutils TREXconfig)
set_source_files_properties(trex_version.cc
  PROPERTIES OBJECT_DEPENDS 
    ${CMAKE_BINARY_DIR}/trex/config/bits/svn/version.hh)



target_link_libraries(TREXutils ${SYSTEM_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
  ${CHRONO_LIB}
  ${Boost_DATE_TIME_LIBRARY}
  ${Boost_THREAD_LIBRARY})
trex_lib(TREXutils core)

set_property(SOURCE private/Pdlfcn.cc 
  PROPERTY COMPILE_DEFINITIONS SUFSHARE="${CMAKE_SHARED_LIBRARY_SUFFIX}")

install(DIRECTORY . DESTINATION include/trex/utils
  FILES_MATCHING PATTERN "*.hh" PATTERN "*.tcc"
  PATTERN "private" EXCLUDE
  PATTERN ".svn" EXCLUDE)
