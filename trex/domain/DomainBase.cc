/** @file DomainBase.cc
 * @brief Basic domain utilities implementation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup domains
 */
#include <sstream>
#include "DomainVisitor.hh"

using namespace TREX::transaction;

std::string DomainExcept::build_message(DomainBase const &d, 
				       std::string const &msg) throw() {
  std::ostringstream oss;
  oss<<"Domain "<<d.getTypeName()<<" "<<d<<" : "<<msg;
  return oss.str();
}

void DomainBase::accept(DomainVisitor &visitor) const {
  visitor.visit(this, true);
}

boost::any DomainBase::getSingleton() const {
  if( !isSingleton() )
    throw DomainAccess(*this, ": not a singleton");
  return singleton();
}

std::string DomainBase::getStringSingleton() const {
  if( !isSingleton() )
    throw DomainAccess(*this, ": not a singleton");
  return stringSingleton();
}
