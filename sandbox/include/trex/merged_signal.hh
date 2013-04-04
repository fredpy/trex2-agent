#ifndef trex_merged_signal_H
# define trex_merged_signal_H


# include "asio_signal.hh"

namespace trex {

  class merged_signal :boost::noncopyable {
  public:
    typedef asio_signal<void (int)>         signal_type;
    typedef signal_type::slot_type          slot_type;
    typedef signal_type::extended_slot_type extended_slot_type;
    
    typedef boost::signals2::connection     connection;
    
    merged_signal(boost::asio::io_service &s);
    ~merged_signal();
    
    void connect_to(signal_type &sig);

    connection connect(slot_type const &slot);
    connection connect_extended(extended_slot_type const &slot);
    
  private:
    class impl;
    
    boost::scoped_ptr<impl> m_impl;
    
    merged_signal();
  }; // trex::merged_signal

} // trex

#endif // trex_merged_signal_H
