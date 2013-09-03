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
#include <boost/iostreams/device/null.hpp>

#include "nice_flags.h"

// disabled daemon option : my current implementation do not work well
// #define DAEMON

using namespace TREX::agent;
using namespace TREX::utils;
namespace tlog=TREX::utils::log;


namespace po=boost::program_options;
namespace pco=po::command_line_style;


namespace {
  
  po::options_description opt("Usage:\n"
                              "  amc <mission>[.cfg] [options]\n\n"
                              "Allowed options");
  
  singleton_use<LogManager> amc_log;
  UNIQ_PTR<Agent> my_agent;
 
}

// Signal and exit handlers

extern "C" {
  
  void amc_cleanup(int sig) {
    singleton_use<LogManager> amc_log;
    
    boost::posix_time::ptime now(boost::posix_time::second_clock::universal_time());
    
    amc_log->syslog("amc", info)<<"============================================";
    amc_log->syslog("amc", info)<<"At "<<boost::posix_time::to_simple_string(now)
    <<" UTC:\n\t - Received signal "<<sig;
    amc_log->syslog("amc", info)<<"============================================";
    amc_log->flush();
    my_agent.reset();
    exit(1);
  }
  
  void amc_terminate() {
    singleton_use<LogManager> amc_log;
    
    amc_log->syslog("amc", error)<<" Received a terminate";
    amc_log->flush();
    abort();
  }
  
}


namespace {
  std::pair<std::string, std::string> reg_threads(std::string const &s) {
    if( s.find("-j")==0 )
      return std::make_pair(s.substr(1, 1), s.substr(2));
    else
      return std::make_pair(std::string(), std::string());
  }
}

int main(int argc, char *argv[]) {
  po::options_description hidden("Hidden options"), cmd_line;
  size_t nice_val = 0, threads = 3;

  // Specifies the handler for extracting the mission name
  hidden.add_options()("mission",
                       po::value<std::string>(),
                       "The name of the mission file");
  po::positional_options_description p;
  p.add("mission", 1); // Only 1 mission file
  
  // Extra options publicly visible
  opt.add_options()
  ("help,h", "produce help message")
  ("version,v", "print trex version")
  ("include-path,I", po::value< std::vector<std::string> >(), "Add a directory to trex search path")
  ("log-dir,L", po::value<std::string>(), "Set log directory")
  ("sim,s", po::value<size_t>()->implicit_value(60),
   "run agent with simulated clock with given deliberation steps per tick")
  ("period,p", po::value<unsigned long>()->implicit_value(1000),
   "run agent with a real time clock with the given period in ms")
  ("nice", po::value<size_t>(&nice_val)->implicit_value(10),
   "run this command with the given nice level")
  ("j", po::value<size_t>(&threads)->implicit_value(3),
   "Set the hnumber of threads (minimum is 3)")
#ifdef DAEMON // fork does not work well with asio apprently ... to be refined
  ("daemon,D", "run amc as a detached daemon (experimental)")
#endif
  ;
 
  // Build the general options
  cmd_line.add(opt).add(hidden);
  
  // This will store the values needed
  po::variables_map opt_val;
  
  try {
    po::store(po::command_line_parser(argc, argv).style(pco::default_style|pco::allow_long_disguise).options(cmd_line).positional(p).extra_parser(&reg_threads).run(),
              opt_val);
    po::notify(opt_val);
  } catch(boost::program_options::error const &e) {
    std::cerr<<"command line error: "<<e.what()<<'\n'
    <<opt<<std::endl;
    exit(1);
  }
  // Deal with informative options
  if( opt_val.count("help") ) {
    std::cout<<"TREX batch execution command.\n"<<opt<<"\nExample:\n  "
    <<"amc sample --sim=50\n"
    <<"  - run trex agent from sample.cfg using a simulated clock with 50 steps per tick\n"<<std::endl;
    exit(0);
  }
  if( opt_val.count("version") ) {
    std::cout<<"amc for trex "<<TREX::version::full_str()<<std::endl;
    exit(0);
  }
  
  // Check that 1 mission is specified
  if( !opt_val.count("mission") ) {
    std::cerr<<"Missing <mission> argument.\n"
             <<opt<<std::endl;
    exit(1);
  }
  
  if( !opt_val.count("nice") ) {
    // I need to check the environment instead
    char *priority_env = getenv("TREX_NICE");
    if( NULL!=priority_env ) {
      try {
        nice_val = boost::lexical_cast<size_t>(priority_env);
      } catch (boost::bad_lexical_cast const &e) {
        std::cerr<<"Ignoring invalid priority $TREX_NICE=\""<<priority_env<<'\"'<<std::endl;
        amc_log->syslog("amc", tlog::error)<<"Ignoring invalid priority $TREX_NICE=\""<<priority_env<<'\"';
      }
    }
  }
  
#ifdef DAEMON
  // Before doing anything else Lets see if we need to start a daemon
  if( opt_val.count("daemon") ) {
    
    pid_t pid = fork();
    
    if( pid<0 ) {
      std::cerr<<"Failed to spawn the daemon process"<<std::endl;
      exit(2);
    }
    if( pid>0 ) {
      // Disable singleton management for this process
      SingletonUse<LogManager>::disable();
      exit(0);
    }
    
    // I am the child : I need to do my cleanup
    setsid();
    umask(0);
    
    // Fork twice to ensurethe process cannot acquire a controlling terminal
    if( (pid = fork()) )  {
      if( pid>0 ) {
        // Disable singleton management for this process
        SingletonUse<LogManager>::disable();
        // I am the parent : I can die
        std::cout<<"Daemon spawned (pid="<<pid<<")\n"
        <<"All messages should now be reported in:\n  ";
        if(opt_val.count("log-dir"))
          std::cout<<opt_val["log-dir"].as<std::string>();
        else
          std::cout<<"$TREX_LOG_DIR/latest";
        std::cout<<"/TREX.log"<<std::endl;
        exit(0);
      } else {
        std::cerr<<"Failed to spawn twice"<<std::endl;
        exit(1);
      }
    }
  
    // I also need to close my standard ouputs
    close(STDIN_FILENO);
    close(STDOUT_FILENO);
    close(STDERR_FILENO);
  }
#endif
  
  // Add first the path given so they will be looked before TREX_PATH
  if( opt_val.count("include-path") ) {
    std::vector<std::string> const &incs = opt_val["include-path"].as< std::vector<std::string> >();
    for(std::vector<std::string>::const_iterator i=incs.begin();
        incs.end()!=i; ++i) {
      if( amc_log->addSearchPath(*i) )
        amc_log->syslog("amc", info)<<"Added \""<<*i<<"\" to search path";
    }
  }
 
  
  // Initialize all the log environment
  if( opt_val.count("log-dir") )
    amc_log->setLogPath(opt_val["log-dir"].as<std::string>());
  // Create log directory and all
  amc_log->logPath();
  

#ifdef DAEMON
  // If I am spawned as daemon reflects it in the logs
  if( opt_val.count("daemon") )
    amc_log->syslog("amc", info)<<"Spawned as daemon with pid="<<getpid();
#endif
  if( 0!=nice_val ) {
    // When nice is not 0 I need to renice the proces
    int ret;
    
#if defined(HAVE_SETPRIORITY)
    int which = PRIO_PROCESS;
    id_t pid;
    
    pid = getpid();
    amc_log->syslog("amc", info)<<"Setting my priority to "<<nice_val;
    ret = setpriority(which, pid, nice_val);
    if( ret<0 ) {
      // No strong failure just log that we failed
      amc_log->syslog("amc", error)<<"Error while trying to set the priority:\n"<<strerror(errno);
    }
#elif defined(HAVE_NICE)
    amc_log->syslog("amc", info)<<"Nicing myself to "<<nice_val;
    
    ret = nice(nice_val);
    if( ret<0 ) {
      // No strong failure just log that we failed
      amc_log->syslog("amc", error)<<"Error while trying to set the priority:\n"<<strerror(errno);
    }
#else
    // No strong failure just log that we don't know how to do that
    amc_log->syslog("amc", error)<<"Don't know how to change nice level on this platform.";
#endif
  }
  
  // Set exit and interruptions handlers
  std::set_terminate(amc_terminate);
  signal(SIGINT, amc_cleanup);
  signal(SIGTERM, amc_cleanup);
  signal(SIGQUIT, amc_cleanup);
  signal(SIGKILL, amc_cleanup);
  // signal(SIGABRT, amc_cleanup);

  // Now I finally reach the more trex specific stuff :
  // - did user asked for a sim clock ?
  clock_ref clk;
  
  // Do we use a simulated clock ?
  if( opt_val.count("period") ) {
    if( opt_val.count("sim") ) {
      std::cerr<<"Options period and sim are conflicting: pick one!\n"
      <<opt<<std::endl;
      exit(1);
    }
    unsigned long ms = opt_val["period"].as<unsigned long>();
    if( ms<=0 ) {
      std::cerr<<"period of "<<ms<<"ms is invalid.\n"
      <<opt<<std::endl;
      exit(1);
    }
    clk.reset(new RealTimeClock(CHRONO::milliseconds(ms)));
  } else if( opt_val.count("sim") ) {
    clk.reset(new StepClock(Clock::duration_type(0),
                            opt_val["sim"].as<size_t>()));
  }
  
  try {
    if( threads>=3 ) {
      amc_log->syslog("amc", tlog::info)<<"Setting thread count to "<<threads;
      amc_log->thread_count(threads, true);
    }
    
    
    // Create the agent
    my_agent.reset(new Agent(opt_val["mission"].as<std::string>(), clk));
    
    // In case we did not have a clock set yet : use a 1Hz rt_clock
    my_agent->setClock(clock_ref(new RealTimeClock(CHRONO::seconds(1))));
    
    // execute the agent
    my_agent->run();
    my_agent.reset(); // destroy
  
    exit(0);
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
  } catch(boost::exception const &be) {
    amc_log->syslog("amc", error)<<"boost exception :"<<boost::diagnostic_information(be);
  } catch(...) {
    // someone threw something which I don't know of
    amc_log->syslog("amc", error)<<"Unknown exception";
    my_agent.reset();
    throw;
  }
  // Should never reach that point
  return -1;
}
