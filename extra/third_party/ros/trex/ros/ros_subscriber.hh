/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2012, MBARI.
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
#ifndef H_trex_ros_ros_subscriber
# define H_trex_ros_ros_subscriber

# include "bits/ros_timeline.hh"

namespace TREX {
  namespace ROS {
    
    /** @brief Observing timeline interface
     *
     * @tparam Message A ROS message type
     *
     * This class acts as an abstract adapter between a ROS subscriber receiving
     * messages of type @p Message and converting them into a timeline.
     *
     * As it is not easy in ROS C++ API to handle this in a generic way we just 
     * provide this class and it is the responsibility of the implementer to 
     * specialize its message handling callback based on the structure of 
     * Message
     *
     * @ingroup ros
     * @author Frederic Py <fpy@mbari.org>
     */
    template<typename Message>
    class ros_subscriber :public details::ros_timeline {
    public:
      ros_subscriber(
      virtual ~ros_subscriber() {}
      
    protected:
      void message(boost::shared_ptr<Message const> const &msg);
      
    private:
                     ::ros::Subscriber m_sub;
      
                    
    }; // TREX::ROS::ros_subscriber<>
    
  } // TREX::ROS
} // TREX

#endif // H_trex_ros_ros_subscriber
