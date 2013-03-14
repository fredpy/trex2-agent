/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2013, MBARI.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TREX Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <trex/agent/RealTimeClock.hh>
#include <trex/agent/Agent.hh>
#include <trex/utils/platform/memory.hh>

#include <boost/python.hpp>

using namespace boost::python;
namespace ta=TREX::agent;
namespace tt=TREX::transaction;
namespace tu=TREX::utils;

namespace {

  bool set_agent_clock(ta::Agent *agent, UNIQ_PTR<ta::Clock> c) {
    if( agent->setClock(c.get()) ) {
      c.release();
      return true;
    }
    return false;
  }
  
}

void export_agent() {
  // Setup my submodule
  object module(handle<>(borrowed(PyImport_AddModule("trex.agent"))));
  scope().attr("agent") = module;
  scope my_scope = module;
  // from now on everything is under trex.agent
 
  
  class_<ta::Clock, UNIQ_PTR<ta::Clock>,
	 boost::noncopyable>("clock", "trex clock abstract class", no_init)
  .def("start", &ta::Clock::doStart)
  .add_property("initial", &ta::Clock::initialTick)
  .add_property("tick", &ta::Clock::tick)
  .def("date_str", &ta::Clock::date_str)
  .def("__str__", &ta::Clock::info)
  ;
  
  class_<ta::RealTimeClock, bases<ta::Clock>, UNIQ_PTR<ta::RealTimeClock>,
	 boost::noncopyable>
  ("rt_clock", "real time clock at 1000Hz resolution",
   init<ta::RealTimeClock::rep const &, optional<unsigned> >(args("period", "percent_use")))
  ;
  
  // implicitly_convertible<UNIQ_PTR<ta::RealTimeClock>, UNIQ_PTR<ta::Clock> >();
  
  class_<ta::Agent, bases<tt::graph>, boost::noncopyable>
  ("agent", "TREX agent class",
   init<tu::Symbol const &, tt::TICK>())
  .def(init<std::string const &>())
  .def(init<boost::property_tree::ptree::value_type &>())
  // need to find a way to take ownership of the clock passed as argument
  .def("set_clock", &set_agent_clock, arg("clock"))
  .add_property("mission_completed", &ta::Agent::missionCompleted)
  .def("initialize", &ta::Agent::initComplete)
  .def("run", &ta::Agent::run)
  .def("step", &ta::Agent::doNext)
  ;
  
  
}
