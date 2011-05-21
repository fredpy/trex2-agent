/** @file "utils/base/ErrnoExcept.cc"
 * @brief ErrnoExcept implementation
 * 
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */
#include <cstring>

#include <sstream>

#include "ErrnoExcept.hh"

using namespace TREX::utils;

/*
 * class ErrnoExcept
 */
// Statics :

std::string ErrnoExcept::build_message(std::string const &from, 
				       std::string const &what) throw() {
  std::ostringstream message;
  
  message<<from<<": \""<<what<<'\"';
  return message.str();
}
  
std::string ErrnoExcept::build_message(std::string const &from) throw() {
  return build_message(from , strerror(errno));
}
