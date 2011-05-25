/* -*- C++ -*- */
/** @file "BasicInterval.hh"
 * @brief Implementation of BasicInterval
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup domains
 */
#include "DomainVisitor.hh"

using namespace TREX::transaction;

// modifiers 

void BasicInterval::completeParsing(rapidxml::xml_node<> const &node) {
  rapidxml::xml_attribute<> *attr = node.first_attribute("min");
  if( NULL!=attr ) {
    std::string tmp(attr->value(), attr->value_size());
    parseLower(tmp);
  }
  attr = node.first_attribute("max");
  if( NULL!=attr ) {
    std::string tmp(attr->value(), attr->value_size());
    parseUpper(tmp);
  }
}

// observers 
    
std::string BasicInterval::getStringLower() const {
  std::ostringstream oss;
  print_lower(oss);
  return oss.str();
}

std::string BasicInterval::getStringUpper() const {
  std::ostringstream oss;
  print_upper(oss);
  return oss.str();
}

std::ostream &BasicInterval::print_domain(std::ostream &out) const {
  if( isSingleton() ) 
    return print_lower(out);
  else
    return print_upper(print_lower(out<<'[')<<", ")<<']';
}

std::ostream &BasicInterval::toXml(std::ostream &out, size_t tabs) const {
  std::fill_n(std::ostream_iterator<char>(out), tabs, ' ');
  out<<'<'<<getTypeName();
  if( hasLower() )
    print_lower(out<<" min=\"")<<'\"';
  if( hasUpper() )
    print_upper(out<<" max=\"")<<'\"';
  return out<<"/>";
}

// manipulators

void BasicInterval::accept(DomainVisitor &visitor) const {
  return visitor.visit(this);
}
