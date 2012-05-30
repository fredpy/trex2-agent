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
#ifndef H_PositionHandler 
# define H_PositionHandler

# include "MessageHandler.hh"
# include "location.hh"

namespace mbari {

  /** @brief Asset position message handler
   *
   * This class handle amqp assets position messages and maintin the position of 
   * these assets as TREX time advance. 
   * New asset discovered result on new timelines created.
   * It also estimates this position as soon as it did received at least two 
   * update for an asset using a linear extrapolation.
   *
   * @author Frederic Py <fpy@mbari.org>
   * @ingroup tracker
   */
  class PositionHandler :public MessageHandler {
  public:
    /** @brief XML constructor
     *
     * Expected XML format: 
     * @code 
     *  <Tracker exchange="<exchange>" route="<route>" />
     * @endcode
     */
    PositionHandler(xml_arg const &arg);
    /** @brief Destructor */
    ~PositionHandler() {}
    
  private:
    bool handleMessage(amqp::queue::message &msg);
    bool synchronize();

    typedef std::pair<bool, location> asset_info;
    typedef std::map<std::string, asset_info> asset_map;
    asset_map m_assets;
    bool m_should_project;
  }; // mbari::PositionHandler

} // mbari 

#endif // H_PositionHandler
