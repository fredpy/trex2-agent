#include <boost/signals2.hpp>

#include <iostream>


class multiplexer :boost::noncopyable {
public:
  multiplexer();
  ~multiplexer();
  
  typedef boost::signals2::signal<void(int)> signal_type;
  
  void connect_to(signal_type &other);
  
  boost::signals2::connection connect(signal_type::slot_type const &slot);
  boost::signals2::connection connect_extended(signal_type::extended_slot_type const &slot);
  
protected:
  class impl;
  
  boost::scoped_ptr<impl> m_impl;
};

void tick(std::string const &who, int val) {
  std::cout<<who<<" tick : "<<val<<std::endl;
}

int main() {
  multiplexer reactor;
  reactor.connect(boost::bind(tick, "reactor", _1));
  
  multiplexer::signal_type a, b;
  
  a.connect(boost::bind(tick, "A", _1));
  b.connect(boost::bind(tick, "B", _1));
  
  reactor.connect_to(a);
  reactor.connect_to(b);
  
  a(0);
  a(1);
  a(2);
  b(0);
  b(1);
  a(3);
  b(0); // A glicth for testing
  b(2);
  b(3);
  a(4);
  b(4);
  
  return 0;
}


//////////// implementation etc

class multiplexer::impl {
public:
  typedef boost::optional<int> sig_holder;
  
  typedef std::map< boost::signals2::connection, sig_holder > conn_map;
  
  typedef multiplexer::signal_type        signal_type;
  typedef signal_type::extended_slot_type slot_type;
  
  
  impl() {}
  ~impl() {}
  
  
  void signal_exec(boost::signals2::connection const &c, int arg) {
    conn_map::iterator pos = m_sigs.find(c);
    if( m_sigs.end()==pos ) {
      // someone tried to emmit to me without connection ... should never happen
      // lets be gentle and just add this guy to my connections
      pos = m_sigs.insert(conn_map::value_type(c, sig_holder(arg))).first;
    } else
      pos->second = arg;
    
    if( fresh(arg) ) {
      // Need to check that everybody is on the same page do it the hard way for now
      for(conn_map::iterator i=m_sigs.begin(); m_sigs.end()!=i;)
        if( i->first.connected() ) {
          if(!i->second)
            return; // I have one connection which has never been set ...
          else
            arg = std::min(arg, *(i->second));
          ++i;
        } else // Not connected anymore => remove it
          m_sigs.erase(i++);
      
      if( fresh(arg) ) {
        m_last = arg;
        m_me(arg);
      }
    }
  }
  
  void connect_to(multiplexer::signal_type &other) {
    if( (&other)!=(&m_me) ) { // would be better to check for complete cycle but that will do for now
      boost::signals2::connection c = other.connect_extended(slot_type(&impl::signal_exec, this, _1, _2));
      boost::optional<int> tmp;
      
      m_sigs.insert(conn_map::value_type(c, tmp));
    }
  }
  
  signal_type &signal() {
    return m_me;
  }
  
private:
  bool fresh(int val) {
    return (!m_last) || val>*m_last;
  }
  
  sig_holder m_last;
  conn_map   m_sigs;
  
  signal_type m_me;
};

multiplexer::multiplexer():m_impl(new impl) {}

multiplexer::~multiplexer() {}


void multiplexer::connect_to(signal_type &other) {
  m_impl->connect_to(other);
}

boost::signals2::connection multiplexer::connect(multiplexer::signal_type::slot_type const &slot) {
  return m_impl->signal().connect(slot);
}

boost::signals2::connection multiplexer::connect_extended(multiplexer::signal_type::extended_slot_type const &slot) {
  return m_impl->signal().connect_extended(slot);
}

