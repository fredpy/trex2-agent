/** @defgroup simcmd sim command
 * @brief A simple debug agent command
 *
 * This module embeds all the code related to the @c sim program 
 *
 * @author Conor McGann @& Frederic Py <fpy@mbari.org>
 * @ingroup commands
 */

/** @file Sim.cc
 * @brief Agent debugging interface
 *
 * This file implements a TREX Agent for interactive execution
 *
 * @author Conor McGann @& Frederic Py <fpy@mbari.org>
 * @ingroup simcmd
 */
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
#include <cctype>

#include <trex/utils/TREXversion.hh>
#include <trex/agent/Agent.hh>
#include <trex/agent/StepClock.hh>
#include <signal.h>

using namespace TREX::agent;
using namespace TREX::utils;
using namespace TREX::transaction;
namespace xml = boost::property_tree::xml_parser;

namespace {

  /** @brief entry point to TREX system log */
  SingletonUse<LogManager> s_log;

  UNIQ_PTR<Agent> my_agent;


  /** @brief @c sim help message
   *
   * This method is invoked each time the user type an
   * invalid command in the @c sim program
   * @ingroup simcmd
   */
  void printHelp() {
    std::cout<<"Options:\n"
	     <<"  Q :- quit\n"
	     <<"  N :- next tick\n"
	     <<"  G :- goto tick (e.g. g100)\n"
	     <<"  P :- post the goals from the attached file\n"
	     <<"       (e.g P goal.req)\n"
	     <<"  K :- kill one reactor (e.g K foo)\n"      
	     // <<"  W :- wrap the agent to tick 0\n"
	     <<"  H :- print this help message"
	     <<std::endl;
  }

  /** @brief Load goals from a file
   * @param name The name of the file
   *
   * This function is called after a @c saim command @c P. It locates
   * the file @a name in @c TREX_PATH, loads its XML content and post
   * all the goals provided in this file.
   *
   * The expected file format is the following:
   * @code
   * <Mission>
   *   <!-- A set of goals -->
   * </Mission>
   * @encode
   * @note the root node is not necessarily named @c Mission and this
   * function just expect the goals losted right below a main root node
   *
   * @sa Agent::sendRequest(Goal &g);
   *
   * @retval true No problem during file loading
   * @retval An error occured
   *
   * @ingroup simcmd
   */
  bool parseGoals(Agent &trex, std::string const &name) {
    bool found;
    std::string file = s_log->locate(name, found).string();

    if( !found ) {
      // try to add the .req extension
      file = s_log->use(name+".req", found);
    } else {
      s_log->use(file, found);
    }

    if( !found ) {
      std::cerr<<"Unable to locate file \""<<name<<"\""<<std::endl;
      s_log->syslog("sim", warn)<<"Unable to find request file \""<<name<<'\"';
      return false;
    } else {
      try {
	std::cout<<"Loading \""<<file<<"\"... "<<std::flush;
	boost::property_tree::ptree config;

	s_log->syslog("sim", info)<<"Loading request file \""<<name<<'\"';
	read_xml(file, config, xml::no_comments|xml::trim_whitespace);
	
	if( config.empty() )
	  std::cout<<"empty file"<<std::endl;
	else {
	  std::cout<<"done\nExtracting goals:"<<std::endl;
	  if( config.size()==1 && !is_tag(config.front(), "Goal") )
	    config = config.front().second;
	  trex.sendRequests(config);
	}
	return true;
      } catch(Exception const &te) {
	s_log->syslog("sim", error)<<"TREX error while loading \""<<name
				   <<"\": "<<te;
	std::cerr<<"TREX error "<<te<<std::endl;
      } catch( std::exception const &e ) {
	s_log->syslog("sim", error)<<"Exception while loading \""<<name
				   <<"\": "<<e.what();
	std::cerr<<"exception: "<<e.what()<<std::endl;
      } catch(...) {
	s_log->syslog("sim", error)<<"Unknwon exception while loading \""
				   <<name<<"\"";
	std::cerr<<"Unknown error"<<std::endl;
      }
      return false;
    }
  }
}

extern "C" {
  void sim_cleanup(int sig) {
    s_log->syslog("sim", info)<<"============================================";
    s_log->syslog("sim", info)<<"Received signal "<<sig; 
    s_log->syslog("sim", info)<<"============================================";
    std::cout<<"Received signal "<<sig<<std::endl;
    my_agent.reset();
    std::cout<<"Goodbye"<<std::endl;
    exit(1);
  }


  void sim_abort(int) {
    s_log->syslog("sim", error)<<"============================================";
    s_log->syslog("sim", error)<<"Received SIGABRT"; 
    s_log->syslog("sim", error)<<"============================================";
    std::cout<<"Received abort."<<std::endl;
    my_agent.reset();
    exit(1);
  } 

  void sim_terminate() {
    std::cerr<<"Program terminated."<<std::endl;
    abort();
  }

}




/** @brief Interactive execution (sim) main function
 * @param argc Number of arguments
 * @param argv command line arguments
 *
 * This is the main function for the @c sim program.
 * This program accepts 1 to 2 arguments :
 * @code
 * sim <mission>[.cfg] [<steps>]
 * @endcode
 * Where :
 * @li @c @<mission@> is the name of a mission configuration file.
 * There should be a file named @c @<mission@>.cfg in the @c TREX_PATH
 * that describes an agent in XML format
 * @c @<steps@> indicates the maximum number of deliberation steps per tick
 * StepClock should allow. If specified then StepClock is the clock used.
 * Otherwise it will load the clock requested in @c @<mission@>.cfg or
 * StepClock with a number of steps set to 60 if none is requested
 *
 * User then enter in some limited interactive shell with the following
 * commands (not case sensitive):
 * @li @c Q quit the program
 * @li @c N execute until next tick
 * @li @c G@<number@> execute until tick @c @<number@>. This tick should
 * be in the future.
 * @li @c P @<file@>[.req] parse @c @<file@> - or @c @<file@>.req if not
 * found - and load the goals attached to it
 * @li @c K @<reactor@> kill the specified reactor
 * @li @c H print help
 * The progam will terminate either on user request (@c Q) or when
 * the mission is completed.
 *
 * @todo We expect on the future to have more advance commands such as
 * breakpoints on observations or the ability to post a goal on
 * a timeline
 *
 * @pre the loaded clock should be interruptible/pausable. Other clocks
 * like RealTimeClock will result on an undefined behavior
 * @ingroup simcmd
 */
int main(int argc, char **argv) {
  std::cout<<"This is TREX v"<<TREX::version::str()<<std::endl;

  if( argc<2 || argc>4 ) {
    std::cerr << "Invalid argument list:"
	      <<"\n Usage: "<<argv[0]<<" configFile [steps]"
	      << std::endl;
    return -1;
  }
  char *configFile = argv[1];
  UNIQ_PTR<Clock> clk;

  int ret = 0;
  try {
    if( argc>=3 )
      clk.reset(new StepClock(Clock::duration_type(0), string_cast<size_t>(argv[2])));
    
    std::set_terminate(sim_terminate);

    // Clean-up the agent on interruptions
    signal(SIGINT, sim_cleanup);
    signal(SIGTERM, sim_cleanup);
    signal(SIGQUIT, sim_cleanup);
    signal(SIGKILL, sim_cleanup);
    signal(SIGABRT, sim_abort);

    my_agent.reset(new Agent(configFile, clk.release(), true));
    my_agent->setClock(new StepClock(Clock::duration_type(0), 60));
    my_agent->initComplete();
    printHelp();
    while( true ) {
      if( my_agent->missionCompleted() ) {
	std::cout<<"Mission completed."<<std::endl;
	s_log->syslog("sim", info)<<"Mission completed.";
	break;
      }
      TICK tick = my_agent->getCurrentTick();
      std::cout<<'['<<my_agent->getName()<<':'<<tick<<"]> ";
      std::string cmdString;
      std::cin>>cmdString;

      char const cmd = std::toupper(cmdString[0]);
      if( 'Q'==cmd ) {
	std::cout<<"Goodbye"<<std::endl;
	s_log->syslog("sim", info)<<"User requested exit.";
	break;
      } else if( 'N'==cmd ) {
	while( my_agent->getCurrentTick()==tick && !my_agent->missionCompleted() )
	  my_agent->doNext();
      } else if( 'G'==cmd ) {
	try {
	  TICK targetTick = string_cast<TICK>(cmdString.substr(1));
	  
	  if( targetTick<=tick ) 
	    std::cout<<"Tick "<<targetTick<<" is in the past."
	      // <<"\nYou can use W to wrap the agent to its initial tick."
		     <<std::endl;
	  else {
	    while( my_agent->getCurrentTick()<targetTick &&
		   !my_agent->missionCompleted() )
	      my_agent->doNext();
	  }  
	}catch(bad_string_cast const &e) {
	  std::cout<<"Ill-formed g command"<<std::endl;
	}
      } else if( 'P'==cmd ) {
	std::string file;
	std::cin>>file;
	while( !file.empty() && std::isspace(file[0]) )
	  file = file.substr(1);
	if( file.empty() ) {
	  std::cerr<<"Missing file name"<<std::endl;
	  printHelp();
	} else if( !parseGoals(*my_agent, file) ) 
	  printHelp();
      } else if( 'K'==cmd ) {
	std::string name;
	std::cin>>name;
	while( !name.empty() && std::isspace(name[0]) )
	  name = name.substr(1);
	if( name.empty() ) {
	  std::cerr<<"Missing reactor name"<<std::endl;
	  printHelp();
	} else {
	  Agent::reactor_iterator pos = my_agent->find_reactor(name);
	  if( my_agent->reactor_end()==pos ) 
	    std::cerr<<"Reactor \""<<name<<"\" not found."<<std::endl;
	  else {
	    my_agent->kill_reactor(*pos);
	    std::cout<<"Reactor \""<<name<<"\" killed."<<std::endl;
	  }
	}
      } else {
        std::cerr<<"Unknown command \""<<cmdString<<"\""<<std::endl;
	printHelp();
      }
    }
  } catch(Exception const &e) {
    std::cerr<<"Caught a TREX exception: "<<e<<std::endl;
    s_log->syslog("sim", error)<<"Exception caught: "<<e;
    ret = -2;
  } catch(std::exception const &se) {
    std::cerr<<"Caught a C++ exception: "<<se.what()<<std::endl;
    s_log->syslog("sim", error)<<"C++ exception caught: "<<se.what();
    ret = -2;
  } catch(...) {
    std::cerr<<"Caught an unknown execption"<<std::endl;
    s_log->syslog("sim", error)<<"Unknow exception exception caught";
    ret = -4;
  }
  s_log->syslog("sim", info)<<"=============================== END sim ============================";
  my_agent.reset();
  return ret;
}

