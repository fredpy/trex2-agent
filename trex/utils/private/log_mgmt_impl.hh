/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Frederic Py
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
#ifndef H_trex_utils_private_log_mgmt_impl
# define H_trex_utils_private_log_mgmt_impl

# include "../log_manager.hh"
# include "../asio_runner.hh"

namespace TREX {
  namespace utils {
    
    
    
    class log_manager::pimpl :boost::noncopyable {
    public:
      pimpl();
      ~pimpl();
      
      log::text_log &syslog() {
        return m_syslog;
      }
      void flush();
      asio_runner &asio() {
        return m_io;
      }
      boost::asio::strand &strand() {
        return m_strand;
      }
      
      bool set_log_path(log_manager::path_type p);
      bool add_search_path(log_manager::path_type p);
      
      log_manager::path_type init();
      bool locate(std::string const &name,
                  log_manager::path_type &dest);
      std::string search_path() const;
      
    private:
      void create_latest();
      void load_search_path();
      
      bool                   m_inited;
      log_manager::path_type m_path;
      asio_runner            m_io;
      log::text_log          m_syslog;
      boost::asio::strand    m_strand;
      
      SHARED_PTR<log::out_file>         m_trex_log;
      
      typedef std::list<log_manager::path_type> path_set;
      path_set m_search_path;
      
      void init_complete();
    }; // TREX::utils::details::log_mgmt_impl
    
  } // TREX::utils
} // TREX

# define TREX_LOG_FILE "TREX.log"
# define TREX_ENV      "TREX_HOME"
# define SEARCH_ENV    "TREX_PATH"
# define LOG_DIR_ENV   "TREX_LOG_DIR"
# define LATEST_DIR    "latest"
# define MAX_LOG_ATTEMPT 1024


#endif // H_trex_utils_private_log_mgmt_impl