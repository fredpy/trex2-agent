cmake_minimum_required(VERSION 2.8)
project(trex-exp)

include(CheckLibraryExists)

check_library_exists(dl dlopen "" LIB_DL)
if(LIB_DL)
  set(SYSTEM_LIBRARIES ${SYSTEM_LIBRARIES} dl)
else(LIB_DL)
  message(FATAL_ERROR "Lib dl not found")
endif(LIB_DL)

check_library_exists(pthread pthread_self "" LIB_PTHREAD)
if(LIB_PTHREAD)
  set(SYSTEM_LIBRARIES ${SYSTEM_LIBRARIES} pthread)
else(LIB_PTHREAD)
  message(FATAL_ERROR "Lib pthread not found")
endif(LIB_PTHREAD)

check_library_exists(rt shm_unlink "" LIB_RT)
if(LIB_RT)
  set(SYSTEM_LIBRARIES ${SYSTEM_LIBRARIES} rt)
endif(LIB_RT)

find_package(Boost 1.47 COMPONENTS 
  system filesystem
  thread date_time)

if(NOT Boost_FOUND)
  message(FATAL_ERROR "Unable to find Boost (>=1.46.1) library")
endif(NOT Boost_FOUND)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(include)

add_executable(multiplexer
	# headers
	 include/trex/asio_runner.hh
	 include/trex/asio_signal_fwd.hh
	 include/trex/asio_signal.hh
	 include/trex/asio_signal_n.hh
	 include/trex/merged_signal.hh
	 include/trex/bits/asio_signal_base.hh
	 include/trex/bits/asio_signal_iter.hh
	 include/trex/bits/asio_signal_template.hh
	# source 
	src/asio_runner.cc
	src/multiplexer.cc 
	src/merged_signal.cc)

target_link_libraries(multiplexer ${SYSTEM_LIBRARIES} 
  ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

source_group("Header\ Files\\bits" FILES
	 include/trex/bits/asio_signal_base.hh
	 include/trex/bits/asio_signal_iter.hh
	 include/trex/bits/asio_signal_template.hh
)
