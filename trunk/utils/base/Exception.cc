/** @file "utils/base/Exception.cc"
 * @brief TREX Exception implementation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */
#include "Exception.hh"

using namespace TREX::utils;

/*
 * class TREX::utils::Exception
 */
// structors :

Exception::Exception(std::string const &msg) throw()
:std::runtime_error(msg) {}

Exception::~Exception() throw() {}

// observers :

std::ostream &Exception::print_to(std::ostream &out) const {
  return out<<what();
}
