/** @defgroup amccmd amc command
 * @brief A simple agent batch running command
 *
 * This module embeds all the code related to the @c amc program 
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

#include "Agent.hh"
#include "RealTimeClock.hh"
#include "StepClock.hh"

using namespace TREX::agent;
using namespace TREX::utils;

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
      clk.reset(new StepClock(0, stepsPerTick));
    } else {
      std::cerr<<"Invalid 2nd argument "<<simOption.str()
	       <<"\nUsage: "<<argv[0]<<" configFile [-sim [steps]]"
	       <<std::endl;
      return -1;
    }
  }
  Agent agent(argv[1], clk.release());
  agent.setClock(new RealTimeClock(1.0));
  agent.run();
  return 0;
}
