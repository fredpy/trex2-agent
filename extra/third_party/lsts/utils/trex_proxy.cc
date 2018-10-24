#include "trex_proxy.hh"

using namespace trex_lsts;
using TREX::transaction::IntegerDomain;

trex_proxy::trex_proxy() {}

trex_proxy::~trex_proxy() {}

std::string trex_proxy::date_str(IntegerDomain::bound const &b) const {
  return date_str(b.value());
}

std::string trex_proxy::duration_str(IntegerDomain::bound const &b) const {
  return duration_str(b.value());
}


