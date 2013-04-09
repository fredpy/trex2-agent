#include "trex/asio_runner.hh"

#include <boost/signals2.hpp>

#include <iostream>
#include <string>

using namespace trex;
namespace ba = boost::asio;
namespace bs = boost::signals2;



class ticker {
public:
  typedef int tick;
  
  typedef bs::signal<void (tick)>           tick_sig;
  typedef ba::deadline_timer::duration_type duration_type;
  
  explicit ticker(ba::io_service &service, duration_type const &f);
  ~ticker();
  
  bool active() const;
  void start(int deadline=0);
  void stop();
  
  void wait_completion();
  
  bs::connection on_tick(tick_sig::slot_type s);
  bs::connection on_end(tick_sig::slot_type s);
  
  void initial_tick(tick v) {
    m_tick(v);
  }
  void completion_task();
  void ticker_fn(boost::system::error_code const &);
  
private:
  ba::io_service &service() {
    return m_strand.get_io_service();
  }
  
  tick m_last_tick, m_deadline;
  
  boost::scoped_ptr< boost::packaged_task<void> > m_terminate_fn;
  boost::unique_future<void> m_completed;
  duration_type const m_freq;
  
  // asio things
  ba::strand         m_strand;
  ba::deadline_timer m_timer;
  boost::scoped_ptr<ba::io_service::work> m_task;
  
  // signals2
  tick_sig m_tick, m_last;
};

ticker::ticker(ba::io_service &service, duration_type const &f)
:m_freq(f), m_strand(service), m_timer(service) {}

ticker::~ticker() {
  stop();
  wait_completion();
}

bool ticker::active() const {
#if 0
  return (!m_completed.is_ready())
  && boost::future_state::uninitialized!=m_completed.get_state();
#else
  return boost::future_state::waiting==m_completed.get_state();
#endif
}

void ticker::start(int deadline) {
  if( deadline>0 && !active() ) {
    m_task.reset(new ba::io_service::work(service()));
    m_last_tick = 0; m_deadline = deadline;
    m_terminate_fn.reset(new boost::packaged_task<void>(boost::bind(&ticker::completion_task, this)));
    m_completed = m_terminate_fn->get_future();
    m_timer.expires_from_now(m_freq);
    m_strand.post(boost::bind(boost::ref(m_tick), 0));
    m_timer.async_wait(m_strand.wrap(boost::bind(&ticker::ticker_fn, this, _1)));
  }
}

void ticker::stop() {
  if( active() ) {
    m_timer.cancel();
  }
}

bs::connection ticker::on_tick(tick_sig::slot_type s) {
  return m_tick.connect(s);
}

bs::connection ticker::on_end(tick_sig::slot_type s) {
  return m_last.connect(s);
}


void ticker::wait_completion() {
  if( active() ) {
    m_completed.wait();
  }
}

void ticker::completion_task() {
  m_task.reset();
  m_last(m_last_tick);
}

void ticker::ticker_fn(boost::system::error_code const &ec) {
  if( boost::asio::error::operation_aborted!=ec &&
     m_last_tick<m_deadline ) {
    m_last_tick += 1;
    m_timer.expires_at(m_timer.expires_at()+m_freq);
    m_timer.async_wait(m_strand.wrap(boost::bind(&ticker::ticker_fn, this, _1)));
    m_tick(m_last_tick);
  } else /* if( m_terminate_fn ) */ {
    (*m_terminate_fn)();
  }
}

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
