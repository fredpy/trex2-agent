/* -*- C++ -*- */
/** @file "trex/utils/log/log_pipe.hh" 
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
#ifndef h_trex_utils_log_log_pipe
# define h_trex_utils_log_log_pipe

# include "text_log.hh"

# include <boost/asio/deadline_timer.hpp>

namespace TREX {
  namespace utils {
    namespace log {

      class log_pipe : boost::noncopyable {
      public:
	log_pipe(text_log &log, std::ostream &os,
		 symbol const &who, symbol const &what=null);
	~log_pipe();
	
	  
      private:
        class pipe_sink {
        public:
          typedef char                       char_type;
          typedef boost::iostreams::sink_tag category;
          
          pipe_sink(log_pipe &master):m_master(&master) {}
          ~pipe_sink() {}
          
          std::streamsize write(char_type const *s, std::streamsize n);

        private:
          log_pipe *m_master;
        };
        
        
        symbol m_who, m_what;
        WEAK_PTR<details::sig_impl>         m_log;
	std::ostream                               &m_dest;
	std::streambuf                             *m_initial;
	std::string                                m_msg;
	boost::iostreams::stream_buffer<pipe_sink> *m_me;
        
        boost::asio::deadline_timer                m_flush;
        
	void flush_msg();
        void flush_to();
        void send(std::string const &msg);
      };

    } // TREX::utils::log
  } // TREX::utils
} // TREX

#endif // h_trex_utils_log_log_pipe
