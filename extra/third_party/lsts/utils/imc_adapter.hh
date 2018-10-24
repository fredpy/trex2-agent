#ifndef H_TREX_dune_imc_adapter
# define H_TREX_dune_imc_adapter

# include <DUNE/Network/UDPSocket.hpp>
# include <DUNE/IO/Poll.hpp>

# include "trex_proxy.hh"
# include "imc_handler.hh"

# include <trex/utils/SingletonUse.hh>
# include <boost/signals2/signal.hpp>


namespace trex_lsts {

  class imc_adapter {
  public:
    typedef DUNE::IMC::Message message;

    typedef boost::signals2::signal<void (SHARED_PTR<message>)> rcv_sig;
    typedef boost::signals2::connection connection;
    typedef rcv_sig::slot_type          slot_type;
    
    
    void set_graph(TREX::transaction::graph &g);
    void set_proxy(trex_proxy *p);
    
    
    bool is_connected() const;
    void init_connection(int local_port, int id, std::string const &remote_addr, int remote_port);
    
    // Sending
    bool send(message &m, std::string const &host, int port);
    bool send(message &m);
    
    int trex_id() const {
      return m_trex_id;
    }
    
    connection connect(slot_type const &handle) {
      return m_rcv.connect(handle);
    }
    
    template<class Msg, class Fn>
    connection typed_connect(Fn f) {
      imc_handler<Msg> handle(f);
      return connect(handle);
    }
    
    
    
  private:
    imc_adapter();
    ~imc_adapter();
    
    typedef boost::asio::io_service::strand strand;
    
    UNIQ_PTR<strand> m_send, m_listen;
    
    DUNE::Network::UDPSocket m_sender, m_receiver;
    DUNE::IO::Poll  m_poll;
    
    
    
    void async_poll();
    void async_send(SHARED_PTR<message> m, std::string host, int port);
    
    
    SHARED_PTR<trex_proxy> m_proxy;
    std::string m_dune_ip;
    int         m_dune_port, m_local_port, m_trex_id;
    
    rcv_sig m_rcv;
    
    
    
    friend class TREX::utils::SingletonWrapper<imc_adapter>;
  };
  
  
  
  
//  class imc_adapter {
//  public:
//    typedef imc_messenger::message message;
//
//    void set_graph(TREX::transaction::graph &g);
//    void set_proxy(trex_proxy *p);
//    void set_trex_id(int id);
//    void set_platform_id(int id);
//
//
//    SHARED_PTR<trex_proxy> proxy() const {
//      return m_proxy;
//    }
//
//    bool bind(int port);
//    bool unbind();
//
//    bool set_dune(std::string const &host, int port);
//
//    SHARED_PTR<message> poll();
//
//
//    bool send(message &m);
//    bool send(message &m, int port, std::string const &host);
//    bool send_via_iridium(message &m, int port, std::string const &host);
//
//
//  private:
//    imc_adapter();
//    ~imc_adapter();
//
//    int iridium_req();
//
//    size_t const c_imc_header_length;
//    size_t const c_max_iridium_payload_length;
//    int m_trex_id, m_platform_id, m_iridium_req;
//
//    typedef DUNE::IMC::TrexAttribute     imc_var;
//    typedef DUNE::IMC::TrexToken         imc_token;
//    typedef TREX::transaction::Variable  trex_var;
//    typedef TREX::transaction::Predicate trex_pred;
//
//    boost::optional<imc_var> variable_to_imc(trex_var const &var) const;
//    imc_token predicate_to_imc(trex_pred const &pred, trex_proxy::tick_type date) const;
//
//
//    SHARED_PTR<trex_proxy> m_proxy;
//    imc_messenger m_messenger;
//
//    friend TREX::utils::SingletonWrapper<imc_adapter>;
//  }; // class trex_lsts::imc_adapter
  
  
} // trex_lsts

#endif // H_TREX_dune_imc_adapter
