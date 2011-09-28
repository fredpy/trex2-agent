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
#ifndef H_Vitre
# define H_Vitre

# include <boost/asio.hpp> 

# include <trex/transaction/TeleoReactor.hh>

namespace TREX {
  namespace vitre {

    /** @brief The vitre interface reactor
     *
     * This class is used as a proxy for the Vitre java interface. It is a
     * simple  reactor that will send through a socket all the observations
     * it received.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup vitre
     */
    class VitreReactor :public TREX::transaction::TeleoReactor {
    public:
      VitreReactor(TREX::transaction::TeleoReactor::xml_arg_type arg);
      ~VitreReactor();

    private:
      void handleInit();
      void handleTickStart();
      void notify(TREX::transaction::Observation const &obs);
      bool synchronize();
      void send(std::string const &str);
      std::string m_host;
      std::string m_port;
      boost::asio::ip::tcp::socket m_socket;

    }; // TREX::vitre::VitreReactor

  } // TREX::vitre
} // TREX

#endif // H_Vitre
