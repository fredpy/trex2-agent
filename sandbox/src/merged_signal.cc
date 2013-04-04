#include "trex/merged_signal.hh"

namespace trex {
  
  class merged_signal::impl {
    typedef merged_signal::signal_type      signal_type;
    typedef merged_signal::connection       connection;
    typedef signal_type::extended_slot_type slot_type;
    
    typedef signal_type::argument_type      argument_type;
    typedef boost::optional<argument_type>  argument_place;

    typedef std::map<connection,argument_place> connection_map;

  public:
    impl(boost::asio::io_service &io):m_signal(io),m_strand(io) {}
    ~impl() {}
    
    void handle_signal(connection c, argument_type arg);
    void connect_to(signal_type &s);
    
    signal_type &signal() {
      return m_signal;
    }
    boost::asio::strand &strand() {
      return m_strand;
    }
  private:
    void check_emit(argument_type last);
    static bool fresh(argument_place p, argument_type arg);
    
    argument_place      m_last_emitted;
    connection_map      m_connections;
    signal_type         m_signal;
    boost::asio::strand m_strand;
  };
  
}

using namespace trex;

/*
 * class trex::merged_signal
 */

// structors

merged_signal::merged_signal(boost::asio::io_service &io)
:m_impl(new merged_signal::impl(io)) {}

merged_signal::~merged_signal() {}


// modifiers

void merged_signal::connect_to(merged_signal::signal_type &s) {
  return m_impl->connect_to(s);
}

merged_signal::connection merged_signal::connect(merged_signal::slot_type const &slot) {
  return m_impl->signal().connect(slot);
}

merged_signal::connection merged_signal::connect_extended(merged_signal::extended_slot_type const &slot) {
  return m_impl->signal().connect_extended(slot);
}

/*
 * class trex::merged_signal::impl
 */

// statics

bool merged_signal::impl::fresh(merged_signal::impl::argument_place p,
                                merged_signal::impl::argument_type arg) {
  return !p || *p<arg;
}

// modifiers

void merged_signal::impl::connect_to(merged_signal::impl::signal_type &s) {
  if( (&s)!=(&m_signal) ) { // simple check to avoid an obvious deadlock
    connection c = s.strand_connect_extended(m_strand,
                                             slot_type(&impl::handle_signal,
                                                this, _1, _2));
    argument_place tmp;
    connection_map::iterator i = m_connections.insert(connection_map::value_type(c, tmp)).first;
    std::cout<<"Added connection "<<(&i->first)<<std::endl;
  }
}

// manipulators

void merged_signal::impl::check_emit(merged_signal::impl::argument_type last) {
  if( fresh(m_last_emitted, last) ) {
    connection_map::iterator i = m_connections.begin();
    
    std::cout<<"\t- "<<last<<" is fresh"<<std::endl;
    
    while (m_connections.end()!=i) {
      if( i->first.connected() ) {
        if( !i->second ) {// one signal is not set => done
          std::cout<<"\t- inactive("<<(&(i->first))<<')'<<std::endl;
          return;
        } else if( *(i->second) < last ) {
          std::cout<<"\t- reduce "<<last<<" to "<<*(i->second)<<std::endl;
          last = *i->second;
        }
        ++i;
      } else
#if 0
        m_connections.erase(i++); // remove dead connection
#else
      {
        std::cout<<"\t- disconnected("<<(&(i->first))<<')'<<std::endl;
        ++i;
      }
#endif
    }
    if( fresh(m_last_emitted, last) ) {
      // Still fresh => emit it
      std::cout<<"\t- emit("<<last<<')'<<std::endl;
      m_last_emitted = last;
      m_signal(last);
    }
  }
}

void merged_signal::impl::handle_signal(merged_signal::impl::connection c, merged_signal::impl::argument_type arg) {
  connection_map::iterator pos = m_connections.find(c);

  // Handle the special case where the connection is not in the map
  if( m_connections.end()==pos ) {
    if( c.connected() )
      pos = m_connections.insert(connection_map::value_type(c, argument_place())).first;
    else
      return; // dead connection => ignore
  }
  std::cout<<(&(pos->first))<<" -> "<<arg<<std::endl;
  
  // At this point I have pos
  if( fresh(pos->second, arg) ) {
    pos->second = arg;
    check_emit(arg);
  }
}


