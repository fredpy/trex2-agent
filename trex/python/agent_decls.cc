/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2015, Frederic Py.
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
#include <trex/agent/StepClock.hh>
#include <trex/agent/Agent.hh>
#include <trex/utils/platform/memory.hh>

#include <boost/python.hpp>

#include "exception_helper.hh"

using namespace boost::python;
namespace ta=TREX::agent;
namespace tt=TREX::transaction;
namespace tu=TREX::utils;

namespace {
  
  tu::SingletonUse<TREX::python::exception_table>  s_py_err;


  bool set_agent_clock(ta::Agent *agent, ta::clock_ref const &c) {
    if( agent->setClock(c) ) {
      return true;
    }
    return false;
  }
  
  typedef CHRONO::duration<double> fl_secs;

  double tick_duration(ta::Clock const &c) {
    return CHRONO::duration_cast<fl_secs>(c.tickDuration()).count();
  }
  
  double to_seconds(ta::Clock const &c, tt::TICK t) {
    return CHRONO::duration_cast<fl_secs>(c.tickDuration()*t).count();
  }
}

void export_agent() {
  // Setup my submodule
  object module(handle<>(borrowed(PyImport_AddModule("trex.agent"))));
  scope().attr("agent") = module;
  scope my_scope = module;
  // from now on everything is under trex.agent
 
  docstring_options doc_options(true, true, false);
  
  module.attr("__doc__") = "Trex agent execution classes.\n"
  "Tha classes that are used to execute a Trex agent as a\n"
  "set of reactors and a clock.\n"
  "The classes in this module reflect the C++ TREXagent library"
  ;
  
  
  class_<ta::Clock, ta::clock_ref,
	 boost::noncopyable>("clock", "trex clock abstract class\n"
                       "This is the class the agent uses measure time as ticks", no_init)
  .def("start", &ta::Clock::doStart, arg("self"),
       "Start the clock. After this call the clock tick will advance")
  .add_property("initial", &ta::Clock::initialTick,
                "initial tick when the clock was started (ofetn 0)")
  .add_property("tick", &ta::Clock::tick,
                "Current tick value\n"
                "NOTE: This value makes sense only after self.start()\n"
                "      was called. Although ti is typically 0 before that.")
  .add_property("tick_duration", &tick_duration,
                "Duration of a tick in seconds")
  .def("date_str", &ta::Clock::date_str, args("self", "tick"),
       "Convert a tick into a human readable date.\n"
       "NOTE: this function output is valid only after self.start()\n"
       "      is called.")
  .def("as_seconds", &to_seconds, args("self", "tick"),
       "Convert a duration in tick into seconds")
  .def("__str__", &ta::Clock::info, arg("self"),
       "Display basic information about this clock.\n")
  ;
  
  class_<ta::Clock::Error, bases<tu::Exception> > clock_e
  ("clock_error", "exception on clock errors", no_init);
  
  s_py_err->attach<ta::Clock::Error>(clock_e.ptr());
  
  
  class_<ta::RealTimeClock, bases<ta::Clock>, SHARED_PTR<ta::RealTimeClock>,
	 boost::noncopyable>
  ("rt_clock", "real time clock at 1000Hz resolution",
   init<ta::RealTimeClock::rep const &,
        optional<unsigned> >(args("self", "period", "percent_use"),
                             "Create a vlock with the given period in ms.\n"
                             "percent_use is internal and indicate how much of a tick\n"
                             "         is allowed for the agent to execute reactors.\n"
                             "          passed this amount the agent will then try to put\n"
                             "          the reactors to sleep\n\n"
                             "Raises:\n"
                             "  trex.utils.exception: either period or percent use\n"
                             "                        is invalid"))
  ;
  
  implicitly_convertible<SHARED_PTR<ta::RealTimeClock>, ta::clock_ref>();
  
  class_<ta::StepClock, bases<ta::Clock>, SHARED_PTR<ta::StepClock>,
         boost::noncopyable>("sim_clock",
                             "Simulated clock.\n"
                             "A clock that do not advance ticks in real-time\n"
                             "and instead advances them in order to simulate\n"
                             "a system where the maximum amount of basic\n"
                             "agent operations is limited to a number of steps\n"
                             "the use of this clock is for simulation and log\n"
                             "replaying purposes",
                             init<unsigned int>(args("self", "nsteps")))
  ;
  
  implicitly_convertible<SHARED_PTR<ta::StepClock>, ta::clock_ref>();
  
  class_<ta::Agent, bases<tt::graph>, boost::noncopyable>
  ("agent", "TREX agent class.\n"
   "The class that represent and maintain a Trex agent as\n"
   "a graph of reactors executed through the rythm of a clock",
   init<tu::Symbol const &, tt::TICK>(args("self", "name", "final_tick"),
                                      "Create a new agent named name and completing its\n"
                                      " lifetime after final_tick"))
  .def(init<std::string const &>(args("self", "cfg_file"),
                                 "Create an agent based on the xml definition in cfg_file"))
  .def(init<boost::property_tree::ptree::value_type &>(args("self",
                                                            "xml_cfg"),
                                                       "Create an agent based on the xml defintion xml_cfg"))
  // need to find a way to take ownership of the clock passed as argument
  .def("set_clock", &set_agent_clock, args("self", "clock"),
       "Set agent clock to clock if the agent did not have a clock yet.\n"
       "return False if the agent already had a clock")
  .add_property("mission_completed", &ta::Agent::missionCompleted,
                "check if the agent lifetime has ended.\n"
                "it can end either because we reached the final tick\n"
                " or the agent has no more reactor")
  .add_property("final_tick", &ta::Agent::finalTick,
                "Final tick after which the agent will complete")
  .def("initialize", &ta::Agent::initComplete, arg("self"),
       "complete agent initialization.\n\n"
       "Raises:\n"
       "  agent_exception: something went wrong during initialization.")
  .def("run", &ta::Agent::run, arg("self"),
       "run the agent until completion.\n\n"
       "Note: run calls initialize on its own so you do not need to\n\n"
       "Raise:\n"
       "   agent_exception: something went wrong during the init\n"
       "   trex.util.exception: something went wrong during the run\n"
       "   ...: any exception that could be trown by a reactor."
       )
  .def("step", &ta::Agent::doNext, arg("self"),
       "run one tick for the agent.\n\n"
       "Note: this method is for simulation purpose and not recommended\n"
       "      to be used. Often it is better to call self.run() instead.\n\n"
       "Raise:\n"
       "   trex.util.exception: something went wrong during the run\n"
       "   ...: any exception that could be trown by a reactor."
       )
  ;
  
  class_<ta::AgentException, bases<tt::GraphException> > agent_e
  ("agent_exception", "Agent related exceptions", no_init);
  
  s_py_err->attach<ta::AgentException>(agent_e.ptr());
  
  class_<ta::CycleDetected, bases<ta::AgentException> > cycle_e
  ("cycle_detected", "detected cyclic dependecy between reactors", no_init);
  
  s_py_err->attach<ta::CycleDetected>(cycle_e.ptr());
  
  
}
