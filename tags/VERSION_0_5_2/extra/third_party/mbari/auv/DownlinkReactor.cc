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
#include "DownlinkReactor.hh"

#include <boost/lambda/lambda.hpp>

#if defined (_WIN32) || defined (__WIN32__) || defined (WIN32) || defined (__CYGWIN__) 
# define SERIAL_PORT_WIN32 
#endif 

#if defined (SERIAL_PORT_WIN32) 
# include <windows.h> 
#else 
# include <termios.h> 
#endif 


using namespace TREX::mbari;
using namespace TREX::transaction;
using namespace TREX::utils; 

namespace {
  
  void flush(boost::asio::serial_port &port, boost::system::error_code &ec) {
#if defined(SERIAL_PORT_WIN32)
    bool is_flushed = ::PurgeComm(port.native_handle(), 
                                  PURGE_RXABORT||PURGE_RXCLEAR||PURGE_TXABORT||PURGE_TXCLEAR);
#else // POSIX !
    bool is_flushed = 0!=::tcflush(port.native_handle(), TCIOFLUSH);
#endif
    if( is_flushed ) 
      ec = boost::system::error_code();
    else {
#if defined(SERIAL_PORT_WIN32)
      ec = boost::system::error_code(::GetLastError(), 
                                     boost::asio::error::get_system_category());
#else
      ec = boost::system::error_code(errno, 
                                     boost::asio::error::get_system_category());
#endif
    }
  }
  
  void flush(boost::asio::serial_port &port) {
    boost::system::error_code ec;
    flush(port, ec);
    if( ec ) 
      throw ec;
  }
  
}

/*
 * class TREX::mbari::DownlinkReactor
 */

// statics 

boost::asio::io_service DownlinkReactor::s_io_service;

// structors 

DownlinkReactor::DownlinkReactor(TeleoReactor::xml_arg_type arg)
:TeleoReactor(arg, false), 
m_modem(s_io_service) {
  
  // Open and set up the serial port
  m_modem.open(parse_attr<std::string>(xml_factory::node(arg), "device"));
  size_t baud_rate = parse_attr<size_t>(9600, xml_factory::node(arg), "baud");
  m_modem.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
  
  init_modem();
  
  
}

DownlinkReactor::~DownlinkReactor() {
  
}

// modifiers 

void DownlinkReactor::handleInit() {
  
}

void DownlinkReactor::handleTickStart() {
  
}

bool DownlinkReactor::synchronize() {
  return true;
}

void DownlinkReactor::init_modem() {
  // clean up the mess
  flush(m_modem);
  // Every command can last up to 5 seconds
  boost::posix_time::milliseconds timer(5000);
  std::string line;
  
  // Send initial commands
  syslog("modem", null)<<"> ATZ0 (reset to profile 0)";
  boost::asio::write(m_modem, boost::asio::buffer("ATZ0\r")); // reset to profile 0
  do {
    if( !read_line(line, timer) )
      throw ReactorException(*this, "modem read did time out");
    if( "ERROR"==line )
      throw ReactorException(*this, "modem returned an ERROR on ATZ0");
  } while( "OK"!=line );
  
                                                            
  syslog("modem", null)<<"> ATE0 (disable echo)";
  boost::asio::write(m_modem, boost::asio::buffer("ATE0\r")); // disable echo
  do {
    if( !read_line(line, timer) )
      throw ReactorException(*this, "modem read did time out");
    if( "ERROR"==line )
      throw ReactorException(*this, "modem returned an ERROR on ATE0");
  } while( "OK"!=line );
  
  std::string model;
  syslog("modem", null)<<"> AT+CGMI (get modem manufacturer)";
  boost::asio::write(m_modem, boost::asio::buffer("AT+CGMI\r")); // disable echo
  do {
    model = line;
    if( !read_line(line, timer) )
      throw ReactorException(*this, "modem read did time out");
    if( "ERROR"==line )
      throw ReactorException(*this, "modem returned an ERROR on AT+CGMI");
  } while( "OK"!=line );
  syslog("modem", null)<<"Model manufacturer is \""<<model<<'\"';
}

std::string DownlinkReactor::read_line() {
  char c;
  std::string result;
  
  for(;;) {
    boost::asio::read(m_modem, boost::asio::buffer(&c, 1));
    switch(c) {
      case '\r':
        break;
      case '\n':
        return result;
      default:
        result += c;
    }
  }
}

bool DownlinkReactor::read_line(std::string &line, 
                                boost::posix_time::milliseconds const &timeout) {
  bool timed_out = false;
  boost::asio::streambuf buff;
  boost::system::error_code ec(boost::asio::error::would_block);
  
  boost::asio::async_read_until(m_modem, buff, "\r\n", 
                                boost::lambda::var(ec)=boost::lambda::_1);
  boost::asio::deadline_timer timer(s_io_service, timeout);
  
  timer.async_wait(boost::lambda::var(timed_out)=true);
  
  do {
    s_io_service.run_one();
    if( timed_out ) {
      m_modem.cancel();
      return false;
    }
  } while( boost::asio::error::would_block==ec );
  timer.cancel();
  
  if( ec ) {
    syslog(error)<<"Serial port error while reading: "<<ec;
    // An error occured during the read
    throw ec;
  }
  std::istream is(&buff);
  std::getline(is, line);
  return true;
}






