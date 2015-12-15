#include <trex/transaction/graph.hh>
#include <trex/transaction/reactor.hh>

#include <iostream>
#include <sstream>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/random.hpp>

#include <trex/utils/timing/asio_tick_timer.hh>

using namespace trex::transaction;
namespace utils=trex::utils;
namespace tlog = trex::utils::log;
namespace bpt=boost::property_tree;



class simple_reactor:public reactor {
public:
  simple_reactor(xml_arg_type &cfg):reactor(cfg),
  rng(CHRONO_NS::chrono::system_clock::now().time_since_epoch().count()+
      reinterpret_cast<long long>(this)){
    syslog(tlog::info)<<"Created";
    provide(name());
  }
  ~simple_reactor() {
    syslog(tlog::info)<<"Destroyed";
  }

private:

  void init_complete() {
    syslog(tlog::info)<<"Init: internal("<<name()<<") == "<<internal(name());
    
    token_ref pred = create_obs(name(), "pred");
    if( pred ) {
      syslog(tlog::info)<<"Posting: "<<*pred<<"[start="<<pred->start()<<"]";
    
      post(pred);
    }
  }
  void new_tick(TICK date) {
    syslog(tlog::info)<<"New tick ["<<date<<"]";
    if( date==1 && !internal("bar") && !internal("bar2") )
      use("bar");
  }
  boost::optional<TICK> synchronize(TICK nxt_s, TICK max_s, ERROR_CODE &ec) {
    int ratio = nxt_s % 3;
    
    
    if( 0==ratio) {
      token_ref pred = create_obs(name(), "pred");
      if( pred ) {
        syslog(tlog::info)<<"Posting: "<<*pred<<"[start="<<pred->start()<<"]";
      
        post(pred);
      }

      syslog(tlog::info)<<"Synchronize("<<nxt_s<<"): DONE";
      return nxt_s;
    } else {
      boost::random::uniform_int_distribution<> coin(0, ratio);
      int val = coin(rng);
      boost::optional<TICK> ret;
      
      
  //    syslog(tlog::info)<<"Coin: "<<val;
   
      std::string info;
      if( 0==val ) {
        ret = nxt_s;
        info = "DONE";
      } else {
        if( val+1>=ratio )
          sleep(1);
        info = "TBC";
      }
      
      syslog(tlog::info)<<"Synchronize("<<nxt_s<<"): "<<info;
      return ret;
    }
  }
  void unsubscribed(utils::symbol tl) {
    if( !internal(tl) ) {
      syslog(tlog::error)<<"trigger isolation.";
      isolate();
    }
  }
  
  void notify(token_id obs) {
    syslog(tlog::info)<<"Notified: "<<(*obs)<<" {start="<<obs->start()
      <<", end="<<obs->end()<<"}";
  }

  
  boost::random::mt19937 rng;
};

reactor::declare<simple_reactor> tmp("simple");

typedef utils::tick_clock<CHRONO_NS::milli> fast_clock;

void print_clock(fast_clock const &c) {
  std::cout<<"Tick "<<c.now().time_since_epoch().count()<<std::endl;
}

int main(int argc, char *argv[]) {
  utils::singleton::use<utils::log_manager> log;
  try {
    utils::log_manager::path_type p = log->log_path();
    
    std::cerr<<"Output logged to "<<p<<std::endl;
  
  
    std::istringstream iss("<Agent name=\"foo\">"
                           "<simple name=\"bar\" lookahead=\"0\" latency=\"1\" />"
                           "<simple name=\"bar2\" lookahead=\"0\" latency=\"1\" />"
                           "<simple name=\"toto\" lookahead=\"0\" latency=\"1\" />"
                           "</Agent>");
  
    bpt::ptree cfg;
    bpt::xml_parser::read_xml(iss, cfg);
    
    graph g(cfg.get_child("Agent"));
    // g.manager().thread_count(6);
  
    std::cout<<"Created agent \""<<g.name()<<"\""<<std::endl;
    g.syslog(tlog::info)<<"Created agent \""<<g.name()<<"\"";
    
    // Make a clock
    utils::asio_tick_timer<fast_clock> timer(g.service());
    
    timer.expires_from_now(fast_clock::duration(100));
    timer.async_wait(boost::bind(&print_clock, boost::ref(timer.clock())));
    
    g.tick(0);
    g.tick(1);
    sleep(1);
    g.tick(2);
    g.tick(2);
    sleep(1);
    g.tick(4);
    sleep(10);
    return 0;
  } catch(std::exception const &e) {
    std::cerr<<"exception: "<<e.what()<<std::endl;
  } catch(...) {
    std::cerr<<"exception: something something"<<std::endl;
  }
  
  return 1;
}