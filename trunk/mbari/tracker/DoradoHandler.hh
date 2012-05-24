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
  
    

  class DoradoHandler :public MessageHandler {
  public:
    DoradoHandler(xml_arg const &arg);
    ~DoradoHandler() {}

  private:
    bool handleMessage(amqp::queue::message &msg);
    bool handleRequest(TREX::transaction::goal_id const &g);
    bool synchronize();
    

    typedef std::map<std::string, double> sensor_data;
    serie<sensor_data> m_serie;
    bool m_updated;
    
    TREX::transaction::Observation m_last_obs;
    bool m_obs_fresh;
    time_t m_since;
    
    
    std::map<std::string, kalman_filter_fn> m_filters;
    
    
    void mbfd(serie<sensor_data>::iterator from, serie<sensor_data>::iterator to,
              std::string const &field);
  }; // mbari::DoradoHandler

} // mbari

#endif // H_DoradoHandler
