/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2011, MBARI.
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
/** @defgroup amccmd amc command
 * @brief A simple agent batch running command
 *
 * This module embeds all the code related to the @c amc program
 *
 * @h1 amc command usage
 *
 * The amc command line is the basic way t execute an agent in batch
 * mode. It can ba e called like this :
 * @code
 * amc <mission>[.cfg] [-sim [steps]
 * @endcode 
 * Where :
 * @li @c @<mission@> is the name of a mission configuration file.
 * There should be a file named @c @<mission@>.cfg in the @c TREX_PATH
 * that describes an agent in XML format:
 * @li @c -sim indicates that we want to use StepClock instead of the
 * default clock. Otherwise it loads the clock loaded by
 * @c @<mission@>.cfg or RealTimeClock if none was required
 * @c @<steps@> indicates the maximum number of deliberation steps per 
 * tick StepClock should allow.
 *
 * The programs locate the file @c <mission> in TREX_PATH -- or 
 * @c <mission>.cfg if @c <mission> is not found -- parse its XML content 
 * in order to initialize the rset of reactors loaded and the potential 
 * clock defined and execute it until completion. If no clock is provided 
 * the command uses a default real-time clock with a tick duration of 1 
 * second.
 *
 * @sa TREX::agent::Agent::Agent(std::string const &, Clock *) 
 *
 * @author Conor McGann @& Frederic Py <fpy@mbari.org>
 * @ingroup commands
 */

/** @file AMC.cc
 * @brief Autonomous Mission Controller
 *
 * This file implements a TREX Agent for Batch mission execution
 *
 * @author Conor McGann @& Frederic Py <fpy@mbari.org>
 * @ingroup amccmd
 */
#include <cstring>
#include <signal.h>

#include <trex/agent/Agent.hh>
#include <trex/agent/RealTimeClock.hh>
#include <trex/agent/StepClock.hh>

#include "nice_flags.h"

using namespace TREX::agent;
using namespace TREX::utils;

namespace {

  std::auto_ptr<Agent> my_agent;
  SingletonUse<LogManager> amc_log;
  
}

extern "C" {

  void amc_cleanup(int sig) {
    amc_log->syslog("amc", info)<<"============================================";
    amc_log->syslog("amc", info)<<"Received signal "<<sig; 
    amc_log->syslog("amc", info)<<"============================================";
    my_agent.reset();
    exit(1);
  }

}


/** @brief Batch execution (amc) main function
 * @param argc Number of arguments
 * @param argv command line arguments
 *
 * This is the main function for the @c amc program.
 * This program accepts 1 to 3 arguments :
 * @code
 * amc <mission>[.cfg] [-sim [<steps>]]
 * @endcode
 * Where :
 * @li @c @<mission@> is the name of a mission configuration file.
 * There should be a file named @c @<mission@>.cfg in the @c TREX_PATH
 * that describes an agent in XML format
 * @li @c -sim indicates that we want to use StepClock instead of the
 * default clock. Otherwise it loads the clock loaded by
 * @c @<mission@>.cfg or RealTimeClock if none was required
 * @c @<steps@> indicates the maximum number of deliberation steps per tick
 * StepClock should allow.
 *
 * This program will then load and execute the agent in a batch mode until
 * its mission is completed or an error occured
 * 
 * @ingroup amccmd
 */
int main(int argc, char **argv) {
  if (argc < 2 || argc > 4) {
    std::cerr << "Invalid argument list:"
	      <<"\n Usage: "<<argv[0]<<" configFile [-sim [steps]]"
	      << std::endl;
    return -1;
  }
  char *configFile = argv[1];
  std::auto_ptr<Clock> clk;
    
  if( argc>=3 ) {
    Symbol simOption = argv[2];
    if( simOption=="-sim" ) {
      size_t stepsPerTick = 60;
      
      if( argc==4 )
	stepsPerTick = string_cast<size_t>(argv[3]);
      clk.reset(new StepClock(Clock::duration_type(0), stepsPerTick));
    } else {
      std::cerr<<"Invalid 2nd argument "<<simOption.str()
	       <<"\nUsage: "<<argv[0]<<" configFile [-sim [steps]]"
	       <<std::endl;
      return -1;
    }
  }
  // Clean-up the agent on interruptions
  signal(SIGINT, amc_cleanup);
  signal(SIGTERM, amc_cleanup);
  signal(SIGQUIT, amc_cleanup);
  signal(SIGKILL, amc_cleanup);
  // force trex log initialization
  amc_log->logPath();

  amc_log->syslog("amc", info)<<"Checking for TREX_NICE";
  char * priority_env = getenv("TREX_NICE");
  if( NULL!=priority_env ) {

    amc_log->syslog("amc", info)<<"Found environment TREX_NICE="<<priority_env;
    unsigned priority;
    try {
      int ret;
      priority = TREX::utils::string_cast<unsigned>(priority_env);
#ifdef HAVE_SETPRIORITY
      int which = PRIO_PROCESS;
      id_t pid;

      pid = getpid();
      amc_log->syslog("amc", info)<<"Setting my priority to "<<priority;
      ret =  setpriority(which, pid, priority);
      if( ret<0 )
	amc_log->syslog("amc", error)<<"Error while trying to set the priority:\n\t"
			    <<strerror(errno);
#else
# ifdef HAVE_NICE
      amc_log->syslog("amc", info)<<"Nicing myself to "<<priority;
      ret = nice(priority);
      if( ret<0 )
	amc_log->syslog("amc", error)<<"Error while trying to set the priority:\n\t"
			    <<strerror(errno);
# else 
      amc_log->syslog("amc", error)<<"Don't know how to change my priority on this system.";
# endif
#endif 
    } catch(TREX::utils::bad_string_cast const &e) {
      amc_log->syslog("amc", error)<<"Failed to parse TREX_NICE as an unsigned:\n"
				   <<e.what();
    }
  }
 
  my_agent.reset(new Agent(configFile, clk.release()));
  // Use a 1Hz clock by default
  my_agent->setClock(new RealTimeClock(CHRONO::seconds(1)));
  try {
    my_agent->run(); 
    my_agent.reset();
    return 0;
  } catch(...) {
    my_agent.reset();
    throw;
  }
}
