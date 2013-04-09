#ifndef trex_ticker_H
# define trex_ticker_H

#include <boost/signals2.hpp>
#include <boost/asio/deadline_timer.hpp>

namespace trex {
  
  class ticker:boost::noncopyable {
  public:
    typedef size_t tick;
    typedef boost::signals2::signal<void (tick)> tick_sig;
    typedef boost::asio::deadline_timer::duration_type duration_type;
    
    ticker(boost::asio::io_service &service,
           duration_type freq);
    ~ticker();
    
    bool active() const;
    void start(int deadline=0);
    void stop();
    void wait_completion();
    
    boost::signals2::connection on_tick(tick_sig::slot_type fn);
    boost::signals2::connection on_end(tick_sig::slot_type fn);
    
  private:
    class impl;
    
    boost::scoped_ptr<impl> m_impl;
    
    ticker(); // No default constructor
  };
  
} // trex

#endif // trex_ticker_H