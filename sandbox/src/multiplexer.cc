#include "trex/merged_signal.hh"

#include <iostream>
#include <string>

using namespace trex;

void tick(std::string const &who, int val) {
  std::cout<<who<<" tick : "<<val<<std::endl;
}


int main() {
  merged_signal reactor;
  reactor.connect(boost::bind(tick, "reactor", _1));
  
  merged_signal::signal_type a, b;
  
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
  b(0); // A glitch for testing
  a(1); // second glitch that should mess me up
  b(2);
  b(3);
  a(4);
  b(4);
  
  return 0;
}


