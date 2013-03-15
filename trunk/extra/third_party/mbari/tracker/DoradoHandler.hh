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
#ifndef H_DoradoHandler 
# define H_DoradoHandler

# include <string>

# include "MessageHandler.hh"
# include "serie.hh"

namespace mbari {
  
  struct kalman_filter_fn {
  public:
    kalman_filter_fn():F(1.0), H(1.0) {}
      
    kalman_filter_fn(double f, double h, double q, double r)
    :F(f), H(h), Q(q), R(r) {}
    
    kalman_filter_fn(kalman_filter_fn const &other) 
    :F(other.F), H(other.H), Q(other.Q), R(other.R) {}
    
    ~kalman_filter_fn() {}
    
    
    std::pair<double, double> operator()(std::pair<double, double> const &p, 
                                         double delta) const {
      double p_minus = (F*p.second*F) + Q;
      double k = (p_minus * H) / (H*p_minus*H + R);
      double f_est = F*p.first;  
      return std::make_pair( f_est + k * (delta - H*f_est),
                            (1.0 - k*H)*p_minus);
    }
    
  private:
    double F;
    double H;
    double Q;
    double R;
  };
  
    
  /** @brief TREX AUV messsage handler
   *
   * This class handle message coming from our TREX controlled AUV. It gives 
   * feedback on the AUV's TREX timelines along with some sensor data and 
   * vehicle position.
   *
   * @author Frederic Py <fpy@mbari.org>
   * @ingroup tracker
   */
  class DoradoHandler :public MessageHandler {
  public:
    /** @brief XML construtor
     *
     * Expected XML:
     * @code
     * <MBFD exchange="<exchange>" route="<route>" />
     * @endcode 
     */
    DoradoHandler(xml_arg const &arg);
    /** @brief Destructor */
    virtual ~DoradoHandler() {}

  private:
    bool handleMessage(amqp::queue::message &msg);
    bool handleRequest(TREX::transaction::goal_id const &g);
    bool synchronize();
    

    typedef std::map<std::string, double> sensor_data;
    serie<sensor_data> m_serie;
    bool m_updated;
    
    TREX::transaction::Observation m_last_obs;
    bool m_obs_fresh;
    date_type m_since;
    
    
    std::map<std::string, kalman_filter_fn> m_filters;
    
    
    void mbfd(serie<sensor_data>::iterator from, serie<sensor_data>::iterator to,
              std::string const &field);
  }; // mbari::DoradoHandler

} // mbari

#endif // H_DoradoHandler
