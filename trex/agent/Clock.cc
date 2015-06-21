/** @file "Clock.cc"
 * @brief Clock class implementation
 *
 * @author Conor McGann
 * @ingroup transaction
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
#include "Clock.hh"

#include <boost/date_time/posix_time/posix_time_io.hpp>

using namespace TREX::agent;
using namespace TREX::transaction;
using namespace TREX::utils;

/*
 * class TREX::transaction::Clock
 */

// statics :
void Clock::sleep(Clock::duration_type const &delay){
  if( delay > Clock::duration_type::zero() ) {
    timespec tv;
    tv.tv_sec = delay.count()/1000000000l;
    tv.tv_nsec = delay.count()%1000000000l;
    while( tv.tv_sec>0 || tv.tv_nsec>0 ) {
      if( 0==nanosleep(&tv, &tv) )
        return;
      if( EINTR!=errno )
        throw ErrnoExcept("Clock:sleep");
    } 
  }
}

// structors :

Clock::~Clock() {
  if( m_data.is_open() ) {
    log_tick();
    m_data<<"</Clock>\n";
    m_data.close();
  }
}

// modifiers :

void Clock::doStart() {
  m_last = 0;
  m_count = 0;
  m_free_count = 0;
  start();
  m_data.open(m_log->file_name("clock.xml").c_str());
  m_data<<"<Clock epoch=\""<<date_str(m_last)<<"\" rate=\""<<tickDuration().count()<<"\" >"<<std::endl;
  
  boost::property_tree::ptree inf;
  inf.add("Info", info());
  std::ostringstream oss;
  
  utils::write_xml(oss, inf);
  m_data<<"  "<<oss.str()<<std::endl;

  m_first = true;
  syslog(TREX::utils::log::info)<<"Clock started at "<<epoch()
  <<' '<<epoch().zone_name()
    <<"\n\t"<<info();
}


void Clock::advanceTick(TICK &tick) {
  ++tick;
}


// manipulators :

TICK Clock::tick() {
  TICK ret = getNextTick();

  if( m_first ) {
    m_first = false;
    m_last = ret;
    m_free = true;
    m_started = true;
    m_count = 0;
    m_free_count = 0;
  } else if( ret!=m_last ) {
    log_tick();
    m_last = ret;
  }
  ++m_count;
  return ret;
}

Clock::duration_type Clock::sleep() {
  if( m_free ) {
    m_free = false;
    m_free_count = m_count;
  }
  return doSleep();
}

Clock::duration_type Clock::doSleep() {
  Clock::duration_type delay = getSleepDelay();
  sleep(delay);
  return delay;
}

TREX::utils::log::stream Clock::syslog(utils::Symbol const &kind) const {
  if( m_started )
    return m_log->syslog(m_last, "clock", kind);
  else 
    return m_log->syslog("clock", kind);
}


// observers :

bool Clock::is_free() const {
  if( m_free ) {
    m_free = free();
    
    if( !m_free )
      m_free_count = m_count;
  }
  return m_free;
}

void Clock::log_tick() const {
  if( m_data.is_open() ) {
    if( m_free )
      m_free_count = m_count;
    m_data<<"  <tick value=\""<<m_last<<"\" count=\""<<m_count
	  <<"\" free=\""<<m_free_count<<"\"/>"<<std::endl;
    m_count = 0;
    m_free_count = 0;
    m_free = true;
  }
}


std::string Clock::date_str(TICK const &tick) const {
  std::ostringstream oss;
  oss<<tick;
  return oss.str();
}

std::string Clock::duration_str(TICK dur) const {
  return date_str(dur);
}

