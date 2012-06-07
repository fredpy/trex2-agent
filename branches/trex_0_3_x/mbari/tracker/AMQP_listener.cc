#include "AMQP_listener.hh"

using namespace mbari::amqp;
using namespace TREX::utils;

listener::listener(queue_ref const &q, listener::handler &handle) 
  :m_running(false), m_queue(q), m_handler(&handle) {}

listener::listener(listener const &other) 
  :m_running(false), m_queue(other.m_queue), m_handler(other.m_handler) {}

listener::~listener() {
  stop();
}

bool listener::is_running() const {
  SharedVar<bool>::scoped_lock lock(m_running);
  return *m_running;
}

void listener::run() {
  m_running = true;
  try {
    while( is_running() ) {
      // std::cerr<<"Listening to queue ... "<<std::endl;
      m_handler->handle(m_queue->consume());
      boost::thread::yield();
    }
  } catch(...) {
    m_running = false;
    throw;
  }
  // std::cerr<<"EOT"<<std::endl;
}

bool msg_buffer::empty() const {
  SharedVar<queue_type>::scoped_lock lock(m_queue);
  return m_queue->empty();
}

boost::shared_ptr<queue::message> msg_buffer::pop() {
  boost::shared_ptr<queue::message> result;

  {
    SharedVar<queue_type>::scoped_lock lock(m_queue);
    if( !m_queue->empty() ) {
      result = m_queue->front();
      m_queue->pop_front();
    }
  }
  return result;
}

void msg_buffer::handle(boost::shared_ptr<queue::message> const &msg) {
    SharedVar<queue_type>::scoped_lock lock(m_queue);
    m_queue->push_back(msg);
}
