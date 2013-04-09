#include <trex/ticker.hh>

#include <boost/asio/strand.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/thread.hpp>

namespace ba=boost::asio;
namespace sig=boost::signals2;

namespace trex {
  
  class ticker::impl {
  public:
    impl(ba::io_service &s, duration_type freq)
    :m_io(s), m_strand(s), m_timer(s), m_freq(freq) {
    }
    
    ~impl() {
      if( active() ) {
        stop();
        wait();
      }
    }
    
    bool active() const;
    void start(int deadline);
    void stop();
    void wait();
    
    ba::strand &strand() {
      return m_strand;
    }   
    tick_sig &on_tick() {
      return m_tick;
    }
    tick_sig &on_end() {
      return m_end;
    }
    
    void complete();
    void tick_impl(boost::system::error_code const &ec);
    
  private:
    ba::io_service                         &m_io;
    ba::strand                              m_strand;
    ba::deadline_timer                      m_timer;
    boost::scoped_ptr<ba::io_service::work> m_running;
    
    duration_type m_freq;
    
    tick_sig m_tick, m_end;
    tick m_last_tick, m_end_tick;
    
    boost::scoped_ptr< boost::packaged_task<void> > m_completed_task;
    boost::unique_future<void> m_completed;
  }; // trex::ticker::impl
  
}

using namespace trex;

/*
 * class trex::ticker
 */

ticker::ticker(ba::io_service &service, ticker::duration_type freq)
:m_impl(new ticker::impl(service, freq))
{}

ticker::~ticker() {}

bool ticker::active() const {
  return m_impl->active();
}

void ticker::start(int deadline) {
  if( deadline>0 ) {
    boost::packaged_task<void> init(boost::bind(&impl::start,
                                                boost::ref(m_impl),
                                                deadline));
    boost::unique_future<void> started = init.get_future();
    m_impl->strand().post(boost::bind(&boost::packaged_task<void>::operator(), boost::ref(init)));
    started.wait();
  }
}

void ticker::stop() {
  if( active() )
    m_impl->stop();
}

void ticker::wait_completion() {
  if( active() )
    m_impl->wait();
}

sig::connection ticker::on_tick(ticker::tick_sig::slot_type fn) {
  return m_impl->on_tick().connect(fn);
}

sig::connection ticker::on_end(ticker::tick_sig::slot_type fn) {
  return m_impl->on_end().connect(fn);
}

/*
 * class trex::ticker::impl
 */

bool ticker::impl::active() const {
  return boost::future_state::waiting==m_completed.get_state();
}


void ticker::impl::start(int deadline) {
  if( !active() ) {
    // Make sure that the service will not die on us
    m_running.reset(new ba::io_service::work(m_io));
    m_last_tick = 0;
    m_end_tick = deadline;
    m_completed_task.reset(new boost::packaged_task<void>(boost::bind(&impl::complete, this)));
    m_completed = m_completed_task->get_future();
    m_strand.post(boost::bind(boost::ref(m_tick), 0));
    m_timer.expires_from_now(m_freq);
    m_timer.async_wait(m_strand.wrap(boost::bind(&impl::tick_impl,
                                                 this, _1)));
  }
}

void ticker::impl::stop() {
  m_timer.cancel();
}

void ticker::impl::wait() {
  m_completed.wait();
}

void ticker::impl::complete() {
  m_running.reset();
  m_end(m_last_tick); // Signal the end of the timer
}

void ticker::impl::tick_impl(boost::system::error_code const &ec) {
  if( boost::asio::error::operation_aborted!=ec &&
      m_last_tick < m_end_tick ) {
    ++m_last_tick;
    m_strand.post(boost::bind(boost::ref(m_tick), m_last_tick));
    m_timer.expires_at(m_timer.expires_at()+m_freq);
    m_timer.async_wait(m_strand.wrap(boost::bind(&impl::tick_impl,
                                                 this, _1)));
  } else
    (*m_completed_task)();
}


