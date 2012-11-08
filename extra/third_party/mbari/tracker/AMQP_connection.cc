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
#include "AMQP_queue.hh"

#include <sstream>

using namespace mbari::amqp;

/*
 * class mbari::amqp::connection
 */ 

// statics

void connection::check_rpc_reply(amqp_rpc_reply_t const &rep,
				 std::string const &context) {
  std::ostringstream msg;
  msg<<context<<": ";
  switch( rep.reply_type ) {
  case AMQP_RESPONSE_NORMAL:
    return;
  case AMQP_RESPONSE_LIBRARY_EXCEPTION:
    switch( rep.reply.id ) {      
    case AMQP_CONNECTION_CLOSE_METHOD:
      {
	amqp_connection_close_t *close = static_cast<amqp_connection_close_t *>(rep.reply.decoded);
	msg<<"Server connection error "<<close->reply_code<<": ";
	msg.write(static_cast<char *>(close->reply_text.bytes),
		  static_cast<size_t>(close->reply_text.len));
      }
      break;
    case AMQP_CHANNEL_CLOSE_METHOD:
      {
	amqp_connection_close_t *close = static_cast<amqp_connection_close_t *>(rep.reply.decoded);
	msg<<"Server channel error "<<close->reply_code<<": ";
	msg.write(static_cast<char *>(close->reply_text.bytes),
		  static_cast<size_t>(close->reply_text.len));
      }
      break;
    default:
      msg<<"Unknown server error (id="<<rep.reply.id<<")";
    }
    break;
  default:
    msg<<"Unknown error !";
  }
  throw error(msg.str());
}

// structors 

connection::connection()
  :m_socket(-1), m_conn(amqp_new_connection()) {}

connection::~connection() {
  if( is_open() )
    close();
  amqp_destroy_connection(m_conn);
}

// observers

void connection::check_rpc_reply(std::string const &context) const {
  check_rpc_reply(amqp_get_rpc_reply(m_conn), context);
}

// modifiers 

void connection::open(std::string const &host, int port) {
  if( is_open() )
    throw error("Connection already opened");
  int fd = amqp_open_socket(host.c_str(), port);
  m_channels = 0;
  m_consumer = -1;
  if( fd<0 ) {
    std::ostringstream oss;
    oss<<"Unable to create a socket connection to "<<host<<':'<<port;
    throw error(oss.str());
  }
  amqp_set_sockfd(m_conn, fd);
  m_socket = fd;
}
 
void connection::login(std::string const &accnt, std::string const &pwd,
		       std::string const &vhost) {
  if( !is_open() ) 
    throw error("atempted to login on a non-open connection");
  amqp_rpc_reply_t rep = amqp_login(m_conn, vhost.c_str(), 0, details::FRAME_MAX, 0,
				    AMQP_SASL_METHOD_PLAIN, accnt.c_str(), pwd.c_str());
  check_rpc_reply(rep, "login");
}

queue_ref connection::create_queue() {
  return queue_ref(new queue(*this));
}

queue_ref connection::create_queue(std::string const &name) {
  return queue_ref(new queue(*this, name));
}

void connection::close() {
  if( is_open() ) {
    amqp_rpc_reply_t rep = amqp_connection_close(m_conn, AMQP_REPLY_SUCCESS);
    m_socket = -1;
    check_rpc_reply(rep, "closing connection");
  }
}

size_t connection::new_channel() {
  size_t chan = m_channels+1;
  amqp_channel_open(m_conn, chan);
  check_rpc_reply("Create channel");
  m_channels = chan;
  return chan;
}

void connection::close_channel(size_t chan) {
  std::ostringstream oss;
  oss<<"Closing channel "<<chan;
  amqp_rpc_reply_t rep = amqp_channel_close(m_conn, chan, AMQP_REPLY_SUCCESS);
  check_rpc_reply(rep, oss.str());
  if( m_channels==chan )
    m_channels = chan-1;
}
