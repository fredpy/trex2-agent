#include <DUNE/IMC/Message.hpp>
#include "imc_handler.hh"

using namespace trex_lsts;
using DUNE::IMC::Message;

basic_imc_handler::basic_imc_handler() {}

basic_imc_handler::basic_imc_handler(uint16_t id):m_filter_id(id) {}

basic_imc_handler::~basic_imc_handler() {}


bool basic_imc_handler::is_filtered() const {
  return m_filter_id.is_initialized();
}

uint16_t basic_imc_handler::filter_id() const {
  return is_filtered()?(*m_filter_id):0;
}

void basic_imc_handler::operator()(SHARED_PTR<Message> msg) {
  if( !m_filter_id || msg->getId()==*m_filter_id )
    do_handle(msg);
}


