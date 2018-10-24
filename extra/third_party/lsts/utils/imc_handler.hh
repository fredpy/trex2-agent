#ifndef H_TREX_dune_imc_handler
# define H_TREX_dune_imc_handler

# include <trex/utils/platform/memory.hh>
# include <boost/function.hpp>
# include <boost/optional.hpp>

namespace DUNE {
  namespace IMC {
    
    class Message;
    
  }
}


namespace trex_lsts {
  
  class basic_imc_handler {
  public:
    ~basic_imc_handler();

    bool is_filtered() const;
    uint16_t filter_id() const;
    
    void operator()(SHARED_PTR<DUNE::IMC::Message> msg);
    
  protected:
    basic_imc_handler();
    explicit basic_imc_handler(uint16_t);

    virtual void do_handle(SHARED_PTR<DUNE::IMC::Message> const &) =0;
    
  private:
    boost::optional<uint16_t> m_filter_id;
  };
  
  
  template<class Msg>
  class imc_handler :public basic_imc_handler {
  public:
    typedef Msg message_type;
    typedef SHARED_PTR<message_type> message_ref;
    
    template<typename Fn>
    explicit imc_handler(Fn f):basic_imc_handler(Msg::getIdStatic()), m_callback(f) {}
    virtual ~imc_handler() {}
    
  private:
    void do_handle(SHARED_PTR<DUNE::IMC::Message> const &m) {
      message_ref real_m = DYNAMIC_PTR_CAST<message_type>(m);
      
      if( real_m )
        m_callback(real_m);
    }
    
    boost::function<void (message_ref)> m_callback;
  };
  
  
  
}

#endif // H_TREX_dune_imc_handler
