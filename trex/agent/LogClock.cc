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
#include "LogClock.hh"

#include <trex/utils/chrono_helper.hh>

#include <boost/date_time/posix_time/posix_time_io.hpp>


using namespace TREX::agent;
using namespace TREX::transaction;
using namespace TREX::utils; 

namespace bpt = boost::property_tree;
namespace xml = boost::property_tree::xml_parser;

namespace {
  Clock::xml_factory::declare<LogClock> decl("LogClock");
}

/*
 * struct TREX::agent::LogClock::tick_info
 */

LogClock::tick_info::tick_info(bpt::ptree::value_type const &node)
  :date(parse_attr<TICK>(node, "value")),
   count(parse_attr<size_t>(node, "count")),
   free_count(parse_attr<size_t>(count, node, "free")) {}

/*
 * class TREX::agent::LogClock 
 */
 
// structors

LogClock::LogClock(bpt::ptree::value_type &node) 
  :Clock(Clock::duration_type::zero()) {
  // find the log to replay
  SingletonUse<LogManager> log;
  std::string file = parse_attr<std::string>("clock.xml", node, "file");
  bool found;
  file = log->use(file, found);
  if( !found )
    throw XmlError(node, "Unable to locate file \""+file+"\"");
  bpt::ptree tks;
  // parse the log
  read_xml(file, tks, xml::no_comments|xml::trim_whitespace);
  if( tks.empty() ) {
    syslog(error)<<"clock log \""<<file<<"\" is empty.";
    throw XmlError(node, "Empty clock log file.");
  }
  if( tks.size()!=1 ) {
    syslog(error)<<"clock log \""<<file<<"\" has more than 1 root.";
    throw XmlError(node, "Invalid clock log file");
  }
  tks = tks.get_child("Clock");
  m_period = Clock::duration_type(parse_attr<Clock::duration_type::rep>(tks, "rate"));
  try {
    m_epoch = parse_attr<Clock::date_type>(tks, "epoch");
  } catch(utils::bad_string_cast const &e) {
    Clock::duration_type dur(m_period);
    dur *= parse_attr<unsigned>(tks, "epoch");
    typedef utils::chrono_posix_convert<Clock::duration_type> cvt;
    m_epoch = boost::posix_time::from_time_t(0);
    m_epoch += cvt::to_posix(dur);
  }
  bpt::ptree::assoc_iterator i, last;
  boost::tie(i, last) = tks.equal_range("tick");
  std::optional<transaction::TICK> prev;
  for(; last!=i; ++i) {
    tick_info tck(*i);
    if( tck.count>0 ) {
      if( prev ) {
        if( tck.date > 1+(*prev) )
          syslog(warn)<<"Missing tick(s) between "<<(*prev)<<" and "<<tck.date;
      }
      prev = tck.date;
      m_ticks.push_back(tck);
    } else
      syslog(warn)<<"Skipping tick "<<tck.date<<" with 0 count.";
  }
  if( m_ticks.empty() )
    throw XmlError(node, "clock log has no valid tick.");
}

// manipulators

TICK LogClock::getNextTick() {
  if( m_ticks.empty() ) {
    syslog(error)<<"no more tick to play.";
    throw Clock::Error("clock has no more tick to play.");
  } 
  tick_info const &tck = m_ticks.front();
  size_t counter = count();


  if( counter< tck.count ) {
    // Note ideally I should be able to skipp anything that occurs after 
    // I am not free anymore and it would be much more fficient but lets 
    // keep it that way for now
    return tck.date;
  } else  {
    TICK expected = tck.date+1, next;

    m_ticks.pop_front();
    if( m_ticks.empty() )
      throw Clock::Error("LogClock has no more tick to play");
    next = m_ticks.front().date;

    if( next!=expected ) {
      syslog(warn)<<"The clock skipped tick(s) between "<<(expected-1)
		  <<" and "<<next;
    }
    return next;
  }
}

// observers

bool LogClock::free() const {
  if( m_ticks.empty() )
    throw Clock::Error("LogClock has no more tick to play");

  return count()<m_ticks.front().free_count;
}

// LogClock::duration_type LogClock::doSleep() {
//   return duration_type();
// }

std::string LogClock::info() const {
  std::ostringstream oss;
  oss<<"LogClock with "<<m_ticks.size();
  display(oss<<" ticks to play\n\tsimulated tick period: ", m_period);
  return oss.str();
}


