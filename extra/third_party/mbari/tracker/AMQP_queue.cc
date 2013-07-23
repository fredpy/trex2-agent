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
#include <sstream>

#include "AMQP_queue.hh"

using namespace mbari::amqp;

// structors 

queue::queue(connection &cn)
  :m_conn(cn), m_channel(cn.new_channel()) {
  amqp_queue_declare_ok_t *r = amqp_queue_declare(m_conn.m_conn, m_channel,
						  amqp_empty_bytes, 0, 0, 0, 1,
						  amqp_empty_table);
  m_conn.check_rpc_reply("Declaring new queue");
  m_name = amqp_bytes_malloc_dup(r->queue);
}

queue::queue(connection &cn, std::string const &name)
  :m_conn(cn), m_channel(cn.new_channel()),
   m_name(amqp_bytes_malloc_dup(amqp_cstring_bytes(name.c_str()))) {
     /* amqp_queue_declare_ok_t *r = */ 
     amqp_queue_declare(m_conn.m_conn, m_channel,
                        m_name, 0, 0, 0, 1,
                        amqp_empty_table);
  m_conn.check_rpc_reply("Declaring queue "+name);
}

queue::~queue() {
  m_conn.close_channel(m_channel);
} 

// modifiers 

void queue::configure(bool no_local, bool no_ack, bool exclusive) {
  amqp_basic_consume(m_conn.m_conn, m_channel, m_name, amqp_empty_bytes,
		     no_local, no_ack, exclusive, amqp_empty_table);
  m_conn.check_rpc_reply("Setting consumption");
}

void queue::bind(std::string const &exchange, std::string const &key) {
  if( exchange.empty() ) 
    throw connection::error("bind: Exchange name cannot be empty");
  else {
    amqp_bytes_t exch, k;
    
    if( key.empty() ) 
      k = amqp_empty_bytes;
    else 
      k = amqp_bytes_malloc_dup(amqp_cstring_bytes(key.c_str()));
    exch = amqp_bytes_malloc_dup(amqp_cstring_bytes(exchange.c_str()));    

    amqp_queue_bind(m_conn.m_conn, m_channel, m_name, exch, k, amqp_empty_table);
    m_conn.check_rpc_reply("bind(\""+exchange+"\", \""+key+"\")");    
  }
}

SHARED_PTR<queue::message> queue::consume() {
  int ret;
  amqp_frame_t             frame;
  amqp_basic_deliver_t    *deliver;
  amqp_basic_properties_t *props;
  SHARED_PTR<message> result;

  // Wait for an header 
  do {
    amqp_maybe_release_buffers(m_conn.m_conn);
    ret = amqp_simple_wait_frame(m_conn.m_conn, &frame);
    if( ret<0 ) {
      std::ostringstream oss;
      oss<<"Error while waiting for a new frame (ret="<<ret<<')';
      throw connection::error(oss.str());
    }
  } while( AMQP_FRAME_METHOD!=frame.frame_type &&
	   AMQP_BASIC_DELIVER_METHOD!=frame.payload.method.id );

  // Extract information about the source of the message
  deliver = (amqp_basic_deliver_t *)frame.payload.method.decoded;
  std::string exchange(static_cast<char *>(deliver->exchange.bytes),
		       deliver->exchange.len), 
    key(static_cast<char *>(deliver->routing_key.bytes), 
	deliver->routing_key.len);
  ret = amqp_simple_wait_frame(m_conn.m_conn, &frame);
  if( ret<0 ) 
    throw connection::error("Error while waiting for header frame");
  if( AMQP_FRAME_HEADER!=frame.frame_type ) 
    throw connection::error("Unexpected non header frame");
  
  // Extract message size from the header 
  props = (amqp_basic_properties_t *)frame.payload.properties.decoded;
  size_t target_size = frame.payload.properties.body_size,
    cur_size = 0;
  
  result.reset(new message(exchange, key, target_size));

  char *bytes = result->m_body;

  while( cur_size<target_size ) {
    ret = amqp_simple_wait_frame(m_conn.m_conn, &frame);
    if( ret<0 ) {
      delete[] bytes;
      throw connection::error("Error while waiting for message body");
    }
    if( AMQP_FRAME_BODY!=frame.frame_type ) {
      delete[] bytes;
      throw connection::error("Unexpected frame while receiving message body");
    }
    char *ptr = (char *)frame.payload.body_fragment.bytes;
    std::copy(ptr, ptr+frame.payload.body_fragment.len, bytes+cur_size);
    cur_size += frame.payload.body_fragment.len;
  }
  return result;
}
