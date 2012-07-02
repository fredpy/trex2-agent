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

add_library(TREXeuropa_core SHARED
  Assembly.cc
  core.cc
  CurrentState.cc
  DeliberationFilter.cc
  europa_helpers.cc
  ModeConstraints.cc
  ReactorConstraints.cc
  Schema.cc
  SynchronizationManager.cc
  TimeConstraints.cc
  # headers 
  ../trex/europa/Assembly.hh
  ../trex/europa/DeliberationFilter.hh
  ../trex/europa/EuropaException.hh
  ../trex/europa/bits/europa_helpers.hh
  ../trex/europa/EuropaPlugin.hh
  ../trex/europa/ModeConstraints.hh
  ../trex/europa/ReactorConstraint.hh
  ../trex/europa/ReactorPropagator.hh
  ../trex/europa/SynchronizationManager.hh
  ../trex/europa/TimeConstraints.hh
)

target_link_libraries(TREXeuropa_core ${EUROPA_LIBRARIES} TREXutils 
  ${Boost_REGEX_LIBRARY})

install(TARGETS TREXeuropa_core DESTINATION lib) 
trex_lib(TREXeuropa_core)
