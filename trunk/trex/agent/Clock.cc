/** @file "Clock.cc"
 * @brief Clock class implementation
 *
 * @author Conor McGann
 * @ingroup transaction
 */
#include "Clock.hh"

using namespace TREX::agent;
using namespace TREX::transaction;
using namespace TREX::utils;

/*
 * class TREX::transaction::Clock
 */

// statics :
void Clock::sleep(double sleepDuration){
  if( sleepDuration>0.0 ) {
    struct timespec tv;
    tv.tv_sec = (time_t) sleepDuration;
    tv.tv_nsec = (long) ((sleepDuration - tv.tv_sec) * 1e+9);
      
    while( tv.tv_sec>0 || tv.tv_nsec>0 ){
      int rval = nanosleep(&tv, &tv);
	
      if(rval == 0) // We are done sleeping
	return;

      if( EINTR!=errno )
	throw ErrnoExcept("Clock::sleep");
    }
  }
}

// modifiers :

void Clock::doStart() {
  start();
}


void Clock::advanceTick(TICK &tick) {
  ++tick;
}


// manipulators :

void Clock::sleep() const {
  sleep(getSleepDelay());
}
