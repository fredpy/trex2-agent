#include <trex/transaction/graph.hh>
#include <trex/transaction/reactor.hh>

#include <iostream>
#include <sstream>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/random.hpp>

#include <trex/config/chrono.hh>

using namespace TREX::transaction;
namespace utils=TREX::utils;
namespace tlog = TREX::utils::log;
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

int main(int argc, char *argv[]) {
  utils::singleton::use<utils::log_manager> log;
  std::cerr<<"Output logged to "<<log->log_path()<<std::endl;
  
  
  std::istringstream iss("<Agent name=\"foo\">"
                         "<simple name=\"bar\" lookahead=\"0\" latency=\"1\" />"
                         "<simple name=\"bar2\" lookahead=\"0\" latency=\"1\" />"
                         "<simple name=\"toto\" lookahead=\"0\" latency=\"1\" />"
                         "</Agent>");
  
  bpt::ptree cfg;
  bpt::xml_parser::read_xml(iss, cfg);
  try {
    utils::singleton::use<utils::log_manager> s_log;
    s_log->log_path();
    

    graph g(cfg.get_child("Agent"));
    // g.manager().thread_count(6);
  
    std::cout<<"Created agent \""<<g.name()<<"\""<<std::endl;
    g.syslog(tlog::info)<<"Created agent \""<<g.name()<<"\"";
    g.tick(0);
    g.tick(1);
    sleep(1);
    g.tick(2);
    g.tick(2);
    sleep(1);
    g.tick(4);
    sleep(10);
  } catch(std::exception const &e) {
    std::cerr<<"exception: "<<e.what()<<std::endl;
  } catch(...) {
    std::cerr<<"exception: something something"<<std::endl;
  }
  
  return 0;
}