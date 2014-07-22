/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Frederic Py.
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
#include "async_ofstream_impl.hh"

namespace asio=boost::asio;
using namespace TREX::utils;

/*
 * class TREX::utils::async_ofstream::pimpl
 */

async_ofstream::pimpl::pimpl(asio::io_service &io,
                             std::string const &path)
:m_io(io), m_file(MAKE_SHARED<std::ofstream>()) {
  get_service().post(boost::bind(&pimpl::open, m_file, path));
}

async_ofstream::pimpl::~pimpl() {
  get_service().post(boost::bind(&std::ofstream::close, m_file));
  m_work.reset();
}

// manipulators

void async_ofstream::pimpl::async_write(std::string const &buffer) {
  get_service().post(boost::bind(&pimpl::write, shared_from_this(), buffer, clock::now()));
}

void async_ofstream::pimpl::write(std::string buffer, clock::time_point tp) {
  
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
