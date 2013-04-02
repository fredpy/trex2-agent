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
#include <trex/utils/TREXversion.hh>

#include <boost/date_time/posix_time/time_formatters.hpp>

#include <boost/program_options.hpp>

#include "nice_flags.h"

using namespace TREX::agent;
using namespace TREX::utils;

namespace po=boost::program_options;
namespace pco=po::command_line_style;

namespace {

  SingletonUse<LogManager> amc_log;
  UNIQ_PTR<Agent> my_agent;
  
  po::options_description opt("Usage:\n"
                              "  amc <mission>[.cfg] [options]\n\n"
                              "Allowed options");
  
}

extern "C" {

  void amc_cleanup(int sig) {
    boost::posix_time::ptime now(boost::posix_time::second_clock::universal_time());

    amc_log->syslog("amc", info)<<"============================================";
    amc_log->syslog("amc", info)<<"At "<<boost::posix_time::to_simple_string(now)
                                <<" UTC:\n\t - Received signal "<<sig;
    amc_log->syslog("amc", info)<<"============================================";
    my_agent.reset();
    exit(1);
  }

  void amc_terminate() {
    amc_log->syslog("amc", error)<<" Received a terminate";
    abort();
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
  size_t stepsPerTick;
  std::string mission_cfg;
  clock_ref clk;
  
  po::options_description hidden("Hidden options"), cmd_line;
  
  // Specifies the handler for extracting the mission name
  hidden.add_options()("mission",                       
                       po::value<std::string>(&mission_cfg),
                       "The name of the mission file");
  po::positional_options_description p;
  p.add("mission", 1); // Only 1 mission file
  
  // Extra options publicly visible
  opt.add_options()
  ("help,h", "produce help message")
  ("version,v", "print trex version")
  ("include-path,I", po::value< std::vector<std::string> >(), "Add a directory to trex search path")
  ("sim,s", po::value<size_t>(&stepsPerTick)->implicit_value(60),
   "run agent with simulated clock with given deliberation steps per tick")
  ("nice", po::value<size_t>()->implicit_value(10),
   "run this command with the given nice level")
  ;
  
  // Build the general options
  cmd_line.add(opt).add(hidden);
  
  po::variables_map opt_val;

  // Extract command line options
  try {
    po::store(po::command_line_parser(argc, argv).style(pco::default_style|pco::allow_long_disguise).options(cmd_line).positional(p).run(),
              opt_val);
    po::notify(opt_val);
  } catch(boost::program_options::error const &e) {
    std::cerr<<"command line error: "<<e.what()<<'\n'
    <<opt<<std::endl;
    return 1;
  }
  
  // Deal with informative options
  if( opt_val.count("help") ) {
    std::cout<<opt<<"\nExample:\n  "
                  <<"amc sample --sim=50\n"
                  <<"  - run trex agent from sample.cfg using a simulated clock with 50 steps per tick\n"<<std::endl;
    return 0;
  }
  if( opt_val.count("version") ) {
    std::cout<<"amc for trex "<<TREX::version::str()<<std::endl;
    return 0;
  }
  
  // Check that one mission was given
  if( !opt_val.count("mission") ) {
    std::cerr<<"No mission specified\n"
    <<opt<<std::endl;
    return 1;
  }
  // Do we use a simulated clock ?
  if( opt_val.count("sim") ) {
    clk.reset(new StepClock(Clock::duration_type(0),
                            opt_val["sim"].as<size_t>()));
  }
  
  // Set exit and interruptions handlers
  std::set_terminate(amc_terminate);
  signal(SIGINT, amc_cleanup);
  signal(SIGTERM, amc_cleanup);
  signal(SIGQUIT, amc_cleanup);
  signal(SIGKILL, amc_cleanup);
  signal(SIGABRT, amc_cleanup);
  
  // Initialize trex log path
  amc_log->logPath();
  
  // Now add all the -I provided
  if( opt_val.count("include-path") ) {
    std::vector<std::string> const &incs = opt_val["include-path"].as< std::vector<std::string> >();
    for(std::vector<std::string>::const_iterator i=incs.begin();
        incs.end()!=i; ++i) {
      if( amc_log->addSearchPath(*i) )
        amc_log->syslog("amc", info)<<"Added \""<<*i<<"\" to search path";
    }
  }
  
  // Last I need to nice the program
  int nice_v = 0; // no nice
  
  if( opt_val.count("nice") ) {
    nice_v = opt_val["nice"].as<size_t>();
  } else {
    // Check environment instead
    amc_log->syslog("amc", info)<<"Checking for TREX_NICE environment variable";
    char *priority_env = getenv("TREX_NICE");
    if( NULL!=priority_env ) {
      amc_log->syslog("amc", info)<<"Found TREX_NICE="<<priority_env;
      try {
        nice_v = string_cast<size_t>(priority_env);
      } catch(bad_string_cast const &e) {
        amc_log->syslog("amc", error)<<"TREX_NICE is not a valid integer";
      }
    }
  }
  
  // If nice_v is jot 0 then attempt to renice this process
  if( nice_v!=0 ) {
    int ret;
    
#if defined(HAVE_SETPRIORITY)
    int which = PRIO_PROCESS;
    id_t pid;
    
    pid = getpid();
    amc_log->syslog("amc", info)<<"Setting my priority to "<<nice_v;
    ret = setpriority(which, pid, nice_v);
    if( ret<0 ) {
      // No strong failure just log that we failed
      amc_log->syslog("amc", error)<<"Error while trying to set the priority:\n"<<strerror(errno);
    }
#elif defined(HAVE_NICE)
    amc_log->syslog("amc", info)<<"Nicing myself to "<<nice_v;
    
    ret = nice(nice_v);
    if( ret<0 ) {
      // No strong failure just log that we failed
      amc_log->syslog("amc", error)<<"Error while trying to set the priority:\n"<<strerror(errno);
    }
#else
    // No strong failure just log that we don't know how to do that
    amc_log->syslog("amc", error)<<"Don't know how to change nice level on this platform.";
#endif
  }
  
  // Finally I can initalize my agent
  my_agent.reset(new Agent(mission_cfg, clk));
  
  // If no clock specified use the degfault 1Hz clock
  my_agent->setClock(clock_ref(new RealTimeClock(CHRONO::seconds(1))));
  
  try {
    // And we can run the agent 
    my_agent->run();
    my_agent.reset();
    return 0;
  } catch(TREX::utils::Exception const &e) {
    // receved a trex error ...
    amc_log->syslog("amc", error)<<"TREX exception :"<<e;
    my_agent.reset();
    throw;
  } catch(std::exception const &se) {
    // received a standard C++ exception
    amc_log->syslog("amc", error)<<"exception :"<<se.what();
    my_agent.reset();
    throw;
  } catch(...) {
    // someone threw something which I don't know of
    amc_log->syslog("amc", error)<<"Unknown exception";
    my_agent.reset();
    throw;
  }  
}
