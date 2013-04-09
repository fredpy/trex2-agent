#include "trex/asio_runner.hh"
#include "trex/ticker.hh"

#include <iostream>
#include <string>

using namespace trex;

void new_tick(std::string name, int v) {
  std::cout<<name<<" tick: "<<v<<" ("<<boost::this_thread::get_id()<<")"<<std::endl;
}

void end_of_clk(int v) {
  std::cout<<"end: "<<v<<" ("<<boost::this_thread::get_id()<<")"<<std::endl;
}

int main() {
  asio_runner io;
  ticker clk(io.service(), boost::posix_time::milliseconds(1));
  
  clk.on_tick(boost::bind(&new_tick, "clk", _1));
  clk.on_end(&end_of_clk);
  
  std::cout<<"Have "<<io.thread_count(5, true)<<" io threads"<<std::endl;
  std::cout<<"Main is "<<boost::this_thread::get_id()<<std::endl;
  
  clk.start(12);
  
#if 0
  sleep(10);
  clk.stop();
#else
  clk.wait_completion();
#endif
  
  
  return 0;
}
