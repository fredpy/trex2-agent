#ifndef H_TREX_dune_private_imc_sender
# define H_TREX_dune_private_imc_sender

# include "../imc_messenger.hh"

# include <queue>
# include <boost/atomic.hpp>
# include <boost/optional.hpp>

# include <trex/utils/SharedVar.hh>

# include <DUNE/Network/UDPSocket.hpp>


namespace trex_lsts {
  
  class imc_messenger::sender :boost::noncopyable {
  public:
    explicit sender(boost::atomic_bool const &b);
    ~sender();
    
    void operator()();
      
    
    bool post(message const &msg, port_type port, std::string const &addr);
    
    
    
  private:
    DUNE::Network::UDPSocket m_sock;
    
    struct send_req {
      send_req(message const &msg, port_type port, std::string const &address);
      send_req(send_req const &other);
      send_req &operator=(send_req const &other);
      ~send_req();
      
      mutable UNIQ_PTR<message> msg;
      port_type port;
      std::string address;
    };
    
    boost::optional<send_req> next();

    typedef std::queue<send_req> send_queue;
    typedef TREX::utils::SharedVar<send_queue> protected_queue;
    protected_queue m_pending;
    boost::atomic_bool const &m_active;
  };
  
}


#endif // H_TREX_dune_private_imc_sender
