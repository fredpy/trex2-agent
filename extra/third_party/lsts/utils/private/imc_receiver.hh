#ifndef H_TREX_dune_private_imc_receiver
# define H_TREX_dune_private_imc_receiver

# include "../imc_messenger.hh"

# include <queue>
# include <boost/atomic.hpp>

# include <trex/utils/SharedVar.hh>

# include <DUNE/Network/UDPSocket.hpp>
# include <DUNE/IO/Poll.hpp>


namespace trex_lsts {

  class imc_messenger::receiver :boost::noncopyable {
  public:
    explicit receiver(port_type p, boost::atomic_bool const &b);
    ~receiver();
    
    void operator()();
    
    bool empty() const;
    message *get();
    
  private:
    typedef std::queue<message *> rcv_queue;
    typedef TREX::utils::SharedVar<rcv_queue> protected_queue;
    
    DUNE::Network::UDPSocket m_socket;
    DUNE::IO::Poll           m_poll;
    mutable protected_queue  m_queue;
    boost::atomic_bool const &m_active;
  };
  
}

#endif // H_TREX_dune_private_imc_receiver
