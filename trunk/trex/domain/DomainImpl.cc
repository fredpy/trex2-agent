/** @file "DomainImpl.cc
 * @brief Various domains implemetation
 *
 * This file implements the code for domains which are part of the TREX
 * core.
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup domains
 */
#include <iterator>
#include <algorithm>

#include "BooleanDomain.hh"
#include "FloatDomain.hh"
#include "IntegerDomain.hh"
#include "StringDomain.hh"

using namespace TREX::transaction;
using namespace TREX::utils;


namespace {
  /** @brief Declaration of the @c int domain
   * @ingroup domains
   */
  DomainBase::xml_factory::declare<IntegerDomain> int_decl  ("int");
  /** @brief Declaration of the @c float domain 
   * @ingroup domains
   */
  DomainBase::xml_factory::declare<FloatDomain>   float_decl("float");
  /** @brief Declaration of the @c bool domain 
   * @ingroup domains
   */
  DomainBase::xml_factory::declare<BooleanDomain> bool_decl ("bool");
 
  /** @brief Declaration of the @c string domain 
   * @ingroup domains
   */
  DomainBase::xml_factory::declare<StringDomain> declStr("string");  
}

Symbol const BooleanDomain::type_name("bool");
Symbol const IntegerDomain::type_name("int");
Symbol const FloatDomain::type_name("float");
Symbol const StringDomain::type_name("string");

BooleanDomain::BooleanDomain(rapidxml::xml_node<> const &node)
  :BasicInterval(node), m_full(true) {
  rapidxml::xml_attribute<> *value = node.first_attribute("value");
  if( NULL!=value ) {
    m_val = TREX::utils::string_cast<bool>(std::string(value->value(), 
						       value->value_size()));
    m_full = false;
  } else 
    completeParsing(node); // just in case someone used min/max
}

std::ostream &BooleanDomain::toXml(std::ostream &out, 
				   size_t tabs) const {
    std::fill_n(std::ostream_iterator<char>(out), tabs, ' ');
    out<<"<bool";
    if( !m_full )
        out<<" value=\""<<m_val<<'\"';
    out<<"/>";
    return out;
}


bool BooleanDomain::intersect(DomainBase const &other) const {
  if( other.getTypeName()!=getTypeName() )
    return false;
  else {
    BooleanDomain const &ref = dynamic_cast<BooleanDomain const &>(other);
    
    return m_full || ref.m_full || m_val==ref.m_val;
  }
}

bool BooleanDomain::equals(DomainBase const &other) const {
  if( other.getTypeName()!=getTypeName() )
    return false;
  else {
    BooleanDomain const &ref = dynamic_cast<BooleanDomain const &>(other);
    if( m_full )
      return ref.m_full;
    else 
      return !ref.m_full && m_val==ref.m_val;
  } 
}
      
DomainBase &BooleanDomain::restrictWith(DomainBase const &other) {
  if( other.getTypeName()!=getTypeName() )
    throw EmptyDomain(*this, "Incompatible types");
  else {
    BooleanDomain const &ref = dynamic_cast<BooleanDomain const &>(other);
    if( m_full ) {
      m_full = ref.m_full;
      m_val = ref.m_val;
    } else if( !ref.m_full && m_val!=ref.m_val ) 
      throw EmptyDomain(*this, "intersection is empty.");
  }
  return *this;
}

