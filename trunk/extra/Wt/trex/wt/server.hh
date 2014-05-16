/* -*- C++ -- */
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
#ifndef H_trex_wt_server
# define H_trex_wt_server

# include <Wt/WServer>

# include <trex/utils/platform/memory.hh>
# include <trex/utils/singleton/use.hh>

namespace TREX {
  namespace wt {
    
    class server :boost::noncopyable {
    public:
      bool is_inited() const;
      bool is_running() const;
      
      /** Initialize the server 
       * @param[in] argc Number of arguments
       * @param[in] argv List of arguments
       *
       * @pre The server was not initialized yet
       *
       * Create a new instance of the Wt server with the given server
       *
       * @post A server is created with either the given arguments or 
       * others if it was initalized before
       */
      void init(size_t argc, char *arg[], boost::optional<std::string> const &cfg);
      /** Start the server
       *
       * @pre init was called at least once
       *
       * Start the exisiting server if tis was not started already
       *
       * @retval true if the server was started
       * @retval fals if the server was already running or there was an error
       *
       * @sa is_inited()
       * @sa is_running()
       */
      bool start();
      
      /** @brief Get server implementation
       *
       * @pre is_inited()
       * @return A reference to the server 
       * @throw std::bad_access Server implementation was not created  
       */
      ::Wt::WServer &impl() const;
      
    private:
      server();
      ~server();
      
      UNIQ_PTR< ::Wt::WServer > m_server;

      friend class TREX::utils::singleton::wrapper<server>;
    };
    
  } // TREX::Wt
} // TREX

#endif // H_trex_wt_server