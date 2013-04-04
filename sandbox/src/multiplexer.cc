#include "trex/merged_signal.hh"
#include "trex/asio_runner.hh"

#include <iostream>
#include <string>

using namespace trex;

void tick(std::string const &who, int val) {
  std::cout<<who<<" tick : "<<val<<std::endl;
}

void tick_incr(merged_signal::signal_type &s, int val) {
  if( val>=4 )
    s(val+1);
}


int main() {
  asio_runner io;
  
  
  merged_signal reactor(io.service());
  reactor.connect(boost::bind(tick, "reactor", _1));
  
  merged_signal::signal_type a(io.service()), b(io.service());
  
  a.connect(boost::bind(tick, "A", _1));
  b.connect(boost::bind(tick, "B", _1));
  
  reactor.connect_to(a);
  reactor.connect_to(b);
  
  reactor.connect(boost::bind(&tick_incr, boost::ref(a), _1));
  a.connect(boost::bind(&tick_incr, boost::ref(b), _1));
  
  io.thread_count(10);
  
  a(0);
  a(1);
  a(2);
  b(0);
  b(1);
  a(3);
  b(0); // A glitch for testing
  a(1); // second glitch that should mess me up
  b(2);
  b(3);
  a(4);
  b(4);
  
  sleep(10);
  io.service().stop();
  
  return 0;
}


