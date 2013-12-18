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


option(ASIO_DEBUG "enable debug of asio" OFF)
mark_as_advanced(ASIO_DEBUG)

configure_file(bits/asio_conf.hh.in
  ${CMAKE_CURRENT_BINARY_DIR}/bits/asio_conf.hh)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/bits/asio_conf.hh
  DESTINATION include/trex/utils/bits
)


include_directories(${CMAKE_CURRENT_BINARY_DIR})




add_library(TREXutils SHARED
  # real source
  ErrnoExcept.cc
  Exception.cc
  LogManager.cc
  Pdlfcn.cc
  PluginLoader.cc
  TREXversion.cc
  XmlUtils.cc
  ptree_io.cc
  asio_runner.cc
  asio_fstream.cc
  log/entry.cc
  log/text_log.cc
  singleton/dummy.cc
  singleton/server.cc
  timing/cpu_clock.cc
  # headers
  asio_fstream.hh
  asio_signal.hh
  asio_signal_n.hh
  asio_signal_fwd.hh
  bits/asio_signal_base.hh
  bits/asio_signal_iter.hh
  bits/asio_signal_template.hh
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
  StringExtract.hh
  Symbol.hh
  TREXversion.hh
  XmlFactory.hh
  XmlUtils.hh
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
  # platform specific information
  platform/chrono.hh 
  platform/memory.hh
  platform/cpp11_deleted.hh
  # template source code
  bits/Factory.tcc
  singleton/bits/use.tcc
  singleton/bits/wrapper.tcc
  bits/Symbol.tcc
  bits/XmlFactory.tcc
)

if(TREX_FROM_PKG)
  # svn data is given by the package
  set(SVN_HH ${CMAKE_SOURCE_DIR}/${TREX_EXTRA_SRC}/svn/version.hh)

  if(EXISTS ${SVN_HH})
    include_directories(${CMAKE_SOURCE_DIR}/${TREX_EXTRA_SRC})
    add_definitions(-DWITH_SVN_TRACK)
  endif()
else(TREX_FROM_PKG)
 # I need to generate the file

 option(WITH_SVN_TRACK "Enable tracking of svn compiled version in TREXversion" ON)
 mark_as_advanced(WITH_SVN_TRACK)

 if(WITH_SVN_TRACK)
    set(SVN_HH svn/version.hh)
    add_custom_target(svnheader ALL 
      DEPENDS ${SVN_HH})
    add_dependencies(trex_prep svnheader)

    # this run my svn script
    add_custom_command(OUTPUT ${SVN_HH}
      PRE_BUILD
      COMMAND ${CMAKE_COMMAND} -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
      -DMY_SVN_TRUNK=trex2-agent.googlecode.com/svn/trunk
      -DOUTPUT_DIR=${TREX_EXTRA_DIR}
      -P ${CMAKE_SOURCE_DIR}/cmake/getsvn.cmake)
    
    include_directories(${TREX_EXTRA_DIR})
    set_source_files_properties(${SVN_HH}
      PROPERTIES GENERATED TRUE
     # HEADER_FILE_ONLY TRUE
     LOCATION ${TREX_EXTRA_DIR}
     )

    add_dependencies(TREXutils svnheader)

    set_source_files_properties(TREXversion.cc
      PROPERTIES OBJECT_DEPENDS ${TREX_EXTRA_DIR}/${SVN_HH})
    add_definitions(-DWITH_SVN_TRACK)
 endif(WITH_SVN_TRACK)
endif(TREX_FROM_PKG)

# map ditectory structure for XCode 
source_group("Source\ Files\\log" FILES 
  log/entry.cc
  log/text_log.cc
)

source_group("Source\ Files\\singleton" FILES 
  singleton/dummy.cc
  singleton/server.cc
)

source_group("Source\ Files\\timing" FILES
  timing/cpu_clock.cc
)

source_group("Header\ Files\\bits" FILES 
  bits/asio_signal_base.hh
  bits/asio_signal_iter.hh
  bits/asio_signal_template.hh
)

source_group("Header\ Files\\platform" FILES
  platform/chrono.hh 
  platform/memory.hh
  platform/cpp11_deleted.hh
)

source_group("Header\ Files\\singleton" FILES 
  singleton/use.hh
)

source_group("Header\ Files\\timing" FILES 
  timing/chrono_helper.hh
  timing/cpu_clock.hh
  timing/tick_clock.hh
  timing/posix_utils.hh
)

source_group("Header\ Files\\singleton\\bits" FILES 
  singleton/bits/dummy.hh
  singleton/bits/server_fwd.hh
  singleton/bits/wrapper.hh
)

source_group("Header\ Files\\singleton\\private" FILES 
  singleton/private/server.hh
)

source_group("Header\ Files\\log" FILES 
  log/log_fwd.hh
  log/entry.hh
  log/stream.hh
  log/text_log.hh
  log/out_file.hh
  log/log_pipe.hh
)

source_group("Header\ Files\\log\\bits" FILES 
  log/bits/log_stream.hh
  log/bits/log_sig.hh
)

source_group("Template Files\\bits" FILES  
  bits/Factory.tcc
  bits/Symbol.tcc
  bits/XmlFactory.tcc
)

source_group("Template Files\\singleton\\bits" FILES  
  singleton/bits/use.tcc
  singleton/bits/wrapper.tcc
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
