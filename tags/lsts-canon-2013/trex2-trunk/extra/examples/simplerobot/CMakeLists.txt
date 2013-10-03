trex_plugin(simplerobot 
 simplerobot_pg.cc SimpleRobot.cc
  # headers
  SimpleRobot.hh
  )
target_link_libraries(simplerobot_pg TREXtransaction)