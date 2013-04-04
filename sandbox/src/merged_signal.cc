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
    impl() {}
    ~impl() {}
    
    void handle_signal(connection const &c, argument_type arg);
    void connect_to(signal_type &s);
    
    signal_type &signal() {
      return m_signal;
    }
  private:
    void check_emit(argument_type last);
    static bool fresh(argument_place p, argument_type arg);
    
    argument_place m_last_emitted;
    connection_map m_connections;
    signal_type    m_signal;
  };
  
}

using namespace trex;

/*
 * class trex::merged_signal
 */

// structors

merged_signal::merged_signal():m_impl(new merged_signal::impl) {}

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
    connection c = s.connect_extended(slot_type(&impl::handle_signal,
                                                this, _1, _2));
    argument_place tmp;
    m_connections.insert(connection_map::value_type(c, tmp));
  }
}

// manipulators

void merged_signal::impl::check_emit(merged_signal::impl::argument_type last) {
  if( fresh(m_last_emitted, last) ) {
    connection_map::iterator i = m_connections.begin();
    
    while (m_connections.end()!=i) {
      if( i->first.connected() ) {
        if( !i->second ) // one signal is not set => done
          return;
        else if( *(i->second) < last )
          last = *i->second;
        ++i;
      } else
        m_connections.erase(i++); // remove dead connection
    }
    if( fresh(m_last_emitted, last) ) {
      // Still fresh => emit it
      m_last_emitted = last;
      m_signal(last);
    }
  }
}

void merged_signal::impl::handle_signal(merged_signal::impl::connection const &c, merged_signal::impl::argument_type arg) {
  connection_map::iterator pos = m_connections.find(c);
  
  // Handle the special case where the connection is not in the map
  if( m_connections.end()==pos ) {
    if( c.connected() )
      pos = m_connections.insert(connection_map::value_type(c, argument_place())).first;
    else
      return; // dead connection => ignore
  }
  // At this point I have pos
  if( fresh(pos->second, arg) ) {
    pos->second = arg;
    check_emit(arg);
  }
}


