/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2013, MBARI.
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
#include "asio_fstream.hh"

#include <boost/bind.hpp>
#include <trex/config/chrono.hh>

#include <fstream>

namespace asio=boost::asio;

namespace TREX {
  namespace utils {
  
    
    class async_ofstream::pimpl:public ENABLE_SHARED_FROM_THIS<async_ofstream::pimpl> {
    public:
      typedef async_ofstream::service service;
      typedef CHRONO::system_clock    clock;
      
      
      pimpl(asio::io_service &io, std::string const &path)
      :m_io(io), m_file(MAKE_SHARED<std::ofstream>()) {
        get_service().post(boost::bind(&pimpl::open, m_file, path));
      }
      ~pimpl() {
        get_service().post(boost::bind(&std::ofstream::close, m_file));
        m_work.reset();
      }
    
      service &get_service() {
        return boost::asio::use_service<service>(m_io);
      }
      
      void async_write(std::string const &buffer) {
        get_service().post(boost::bind(&pimpl::write, shared_from_this(), buffer, clock::now()));
      }
      void write(std::string buffer, clock::time_point tp) {
        
        m_file->write(buffer.c_str(), buffer.length());
        if( !m_last ) {
          m_last = tp;
        }

        CHRONO::milliseconds const limit(10), // force flush only at ~100Hz
          delta = CHRONO::duration_cast<CHRONO::milliseconds>(clock::now()-*m_last);
        if( delta>limit ) {
          std::flush(*m_file);
          m_last = tp;
        }
      }
      
      static void open(SHARED_PTR<std::ofstream> f, std::string p) {
        f->open(p.c_str());
      }
      
      void set_work(SHARED_PTR<asio::io_service::work> wk) {
        m_work = wk;
      }
      
    private:
      asio::io_service   &m_io;
      SHARED_PTR<std::ofstream> m_file;
      SHARED_PTR<asio::io_service::work>    m_work;
      
      boost::optional<clock::time_point> m_last;
    };
        
  }
}

using namespace TREX::utils;

/*
 * class TREX::utils::async_ofstream
 */

void async_ofstream::open(std::string const &fname) {
  m_impl = MAKE_SHARED<pimpl>(boost::ref(m_io), fname);
  m_impl->get_service().async_reserve(boost::bind(&pimpl::set_work, m_impl, _1));
}

void async_ofstream::close() {
  m_impl.reset();
}

async_ofstream::entry async_ofstream::new_entry() {
  return entry(m_impl);
}

/*
 * class TREX::utils::async_ofstream::entry_sink
 */
async_ofstream::entry_sink::entry_sink() {}

async_ofstream::entry_sink::entry_sink(SHARED_PTR<async_ofstream::pimpl> const &dest)
:m_dest(dest) {}

async_ofstream::entry_sink::entry_sink(async_ofstream::entry_sink const &other) {
  std::swap(m_dest, other.m_dest);
  std::swap(m_cache, other.m_cache);
}

void async_ofstream::entry_sink::flush() {
  if( m_dest && !m_cache.empty() ) {
    std::string tmp;
    std::swap(tmp, m_cache);
    m_dest->async_write(tmp);
  }
}

std::streamsize async_ofstream::entry_sink::write(async_ofstream::entry_sink::char_type const *s, std::streamsize n) {
  if( m_dest ) {
    m_cache.append(s, n);
    return n;
  }
  return 0;
}

/*
 * class async_oftsream::entry
 */
async_ofstream::entry::entry(async_ofstream::entry const &other)
:m_out(STD_MOVE(other.m_out)) {}

async_ofstream::entry::entry(SHARED_PTR<async_ofstream::pimpl> const &dest)
:m_out(new entry_impl(entry_sink(dest))) {}

std::ostream &async_ofstream::entry::stream() {
  return *m_out;
}

void async_ofstream::entry::flush() {
  if( NULL!=m_out.get() ) {
    (*m_out)->flush();
  }
}

/*
 * class async_buffer_sink
 */

std::streamsize async_buffer_sink::write(async_buffer_sink::char_type const *s, std::streamsize n) {
  // Find last '\n'
  std::streamsize pos = n;
  for( ; pos>0 && '\n'!=s[pos-1]; --pos);
  
  if( pos>0 ) {
    m_dest.write(s, pos);
    m_dest.flush();
  }
  m_dest.write(s+pos, n-pos);
  return n;
}








