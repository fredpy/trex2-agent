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
#ifndef H_trex_utils_private_async_ofstream_impl
# define H_trex_utils_private_async_ofstream_impl

#include "../asio_fstream.hh"
#include "../bits/stranded_service.hh"

#include <trex/config/chrono.hh>
#include <boost/optional.hpp>

namespace TREX {
  namespace utils {
    
    class async_ofstream::pimpl
    :public ENABLE_SHARED_FROM_THIS<async_ofstream::pimpl> {
    public:
      typedef stranded_service<pimpl> service_type;
      typedef CHRONO::system_clock    clock;
      
      pimpl(boost::asio::io_service &io, std::string const &path);
      ~pimpl();
      
      service_type &get_service() {
        return boost::asio::use_service<service_type>(m_io);
      }
      
      void async_write(std::string const &buffer);
      void write(std::string buffer, clock::time_point tp);
      
      static void open(SHARED_PTR<std::ofstream> f, std::string p) {
        f->open(p.c_str());
      }
      
      void set_work(SHARED_PTR<boost::asio::io_service::work> wk) {
        m_work = wk;
      }
      
    private:
      boost::asio::io_service   &m_io;
      SHARED_PTR<std::ofstream> m_file;
      SHARED_PTR<boost::asio::io_service::work>    m_work;
      
      boost::optional<clock::time_point> m_last;
    };
    
  } // TREX::utils
} // TREX

#endif // H_trex_utils_private_async_ofstream_impl