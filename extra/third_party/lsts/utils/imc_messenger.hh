#ifndef H_TREX_dune_imc_messenger
# define H_TREX_dune_imc_messenger

# include <string>
# include <boost/thread/thread.hpp>
# include <boost/atomic.hpp>

# include "imc_handler.hh"
# include "trex_proxy.hh"

namespace trex_lsts {

  class imc_messenger {
  public:
    typedef int port_type;
    typedef DUNE::IMC::Message message;
    
    imc_messenger();
    virtual ~imc_messenger();
    
    void start_listen(port_type bind_p);
    void stop_listen();
    
    bool inbox_empty() const;
    message *receive();
    
    void post(message const &msg, port_type port, std::string const &address);
    
  private:
    class sender;
    class receiver;
    
    boost::atomic_bool m_listen, m_send;
    
    UNIQ_PTR<sender> m_sender;
    UNIQ_PTR<receiver> m_receiver;
    
    UNIQ_PTR<boost::thread> m_send_th, m_listen_th;
  }; // class trex_lsts::imc_messenger
  
  
} // trex_lsts

#endif // H_TREX_dune_imc_messenger
