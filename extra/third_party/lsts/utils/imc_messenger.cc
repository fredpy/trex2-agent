#include "private/imc_sender.hh"
#include "private/imc_receiver.hh"

# include <DUNE/IMC/Message.hpp>

using namespace trex_lsts;

/*
 * class trex_lsts::imc_messenger::sender::send_req
 */

// structors

imc_messenger::sender::send_req::send_req(imc_messenger::message const &mesg, imc_messenger::port_type p, std::string const &addr)
:msg(mesg.clone()), port(p), address(addr) {}

imc_messenger::sender::send_req::send_req(imc_messenger::sender::send_req const &other)
:msg(other.msg.release()), port(other.port), address(other.address) {}

imc_messenger::sender::send_req::~send_req() {}



imc_messenger::sender::send_req &imc_messenger::sender::send_req::operator=(send_req const &other) {
  msg.reset(other.msg.release());
  port = other.port;
  address = other.address;
  return *this;
}


/*
 * class trex_lsts::imc_messenger::sender
 */

// structors

imc_messenger::sender::sender(boost::atomic_bool const &b): m_active(b) {
}

imc_messenger::sender::~sender() {}

// modifiers

bool imc_messenger::sender::post(message const &msg, port_type port, std::string const &addr) {
  if( m_active.load() ) {
    protected_queue::scoped_lock guard(m_pending);
    m_pending->push(send_req(msg, port, addr));
    return true;
  } else
    return false;
}

boost::optional<imc_messenger::sender::send_req> imc_messenger::sender::next() {
  boost::optional<send_req> ret;
  
  protected_queue::scoped_lock guard(m_pending);
  if( !m_pending->empty() ) {
    ret = m_pending->front();
    m_pending->pop();
  }
  return ret;
}



void imc_messenger::sender::operator()() {
  while( m_active.load() ) {
    boost::this_thread::sleep(boost::posix_time::milliseconds(50));
    boost::optional<send_req> cur = next();
    
    if( cur.is_initialized() ) {
      DUNE::Utils::ByteBuffer buff;
      try {
        DUNE::IMC::Packet::serialize(cur->msg.get(), buff);
        m_sock.write(buff.getBuffer(), cur->msg->getSerializationSize(),
                     DUNE::Network::Address(cur->address.c_str()), cur->port);
      } catch(std::runtime_error const &e) {
        std::cerr<<"Error while sending message to Dune: "<<e.what()<<std::endl;
      }
    }
  }
}

/*
 * class trex_lsts::imc_messenger::receiver
 */

imc_messenger::receiver::receiver(imc_messenger::port_type p, boost::atomic_bool const &b):m_active(b) {
  m_socket.bind(p, DUNE::Network::Address::Any, true);
  m_poll.add(m_socket);
}

imc_messenger::receiver::~receiver() {
  protected_queue::scoped_lock guard(m_queue);
  while( !m_queue->empty() ) {
    UNIQ_PTR<message> tmp(m_queue->front());
    m_queue->pop();
  }
}

// observers

bool imc_messenger::receiver::empty() const {
  protected_queue::scoped_lock guard(m_queue);
  return m_queue->empty();
}

// modifiers

void imc_messenger::receiver::operator()() {
  uint8_t buff[65535];
  
  while( m_active.load() ) {
    if( m_poll.poll(m_socket, 100) ) {
      DUNE::Network::Address addr;
      
      uint16_t rv = m_socket.read(buff, 65535, &addr);
      UNIQ_PTR<message> msg(DUNE::IMC::Packet::deserialize(buff, rv));
      if( msg ) {
        protected_queue::scoped_lock guard(m_queue);
        m_queue->push(msg.release());
      }
    }
  }
}

imc_messenger::message *imc_messenger::receiver::get() {
  protected_queue::scoped_lock guard(m_queue);
  if( m_queue->empty() )
    return nullptr;
  else {
    message *ret = m_queue->front();
    m_queue->pop();
    return ret;
  }
}

/*
 * class trex_lsts::imc_messenger
 */

// structors

imc_messenger::imc_messenger():m_listen(false), m_send(false) {
}

imc_messenger::~imc_messenger() {
  stop_listen();
  if( m_send.exchange(false) ) {
    m_send_th->join();
  }
}

// observers

bool imc_messenger::inbox_empty() const {
  return ( !m_receiver )|| m_receiver->empty();
}

// manipualtors

void imc_messenger::start_listen(port_type bind_p) {
  if( !m_listen.exchange(true) ) {
    m_receiver.reset(new receiver(bind_p, m_listen));
    m_listen_th.reset(new boost::thread(boost::bind(&receiver::operator(), m_receiver.get())));
  }
}

void imc_messenger::stop_listen() {
  if( m_listen.exchange(false) ) {
    m_listen_th->join();
    m_listen_th.reset();
    m_receiver.reset();
  }
}

imc_messenger::message *imc_messenger::receive() {
  if( m_receiver )
    return m_receiver->get();
  else
    return nullptr;
}

void imc_messenger::post(message const &msg, port_type port, std::string const &address) {
  if( !m_send.exchange(true) ) {
    m_sender.reset(new sender(m_send));
    m_send_th.reset(new boost::thread(boost::bind(&sender::operator(), m_sender.get())));
  }
  m_sender->post(msg, port, address);
}


