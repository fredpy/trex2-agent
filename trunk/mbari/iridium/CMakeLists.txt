  # Check for Poco
  find_path(POCO_INCLUDE_DIR Poco/Poco.h 
    PATHS /usr/local/include 
    /usr/include 
    /opt/local/include
    )
  find_library(POCO_NET_LIBRARY
    NAMES PocoNet
    PATHS /usr/local/lib 
    /usr/lib 
    /opt/local/lib
    )

if(POCO_NET_LIBRARY)
  include_directories(${POCO_INCLUDE_DIR})
  add_library(MBARIiridium SHARED
    SbdMailer.cc
    DTAReactor.cc
    #headers 
    SbdMailer.hh
    DTAReactor.hh
    )
  target_link_libraries(MBARIiridium 
    TREXtransaction
    ${POCO_NET_LIBRARY})

  install(TARGETS MBARIiridium DESTINATION lib)    
  trex_lib(MBARIiridium)

  trex_plugin(iridium iridium_pg.cc)
  target_link_libraries(iridium_pg MBARIiridium)


  # A test program
  add_executable(sender_test test/sender_test.cc)
  target_link_libraries(sender_test MBARIiridium)
  add_custom_target(test COMMENT "Simple tests programs")
  add_dependencies(test sender_test)
  
else(POCO_NET_LIBRARY)
  message(SEND_ERROR "Cannot compile MBARI tracker without Poco")
endif(POCO_NET_LIBRARY)
