/* -*- C++ -*- */
/** @file "BasicEnumerated.hh"
 * @brief Implementation of BasicEnumerated
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup domains
 */
#include <sstream>

#include "rapidxml_print.hpp"
#include "DomainVisitor.hh"

using namespace TREX::transaction;

/*
 * class TREX::transaction::BasicEnumerated
 */

// structors 

// modifiers :

void BasicEnumerated::completeParsing(rapidxml::xml_node<> const &node) {
  rapidxml::xml_node<> *iter = node.first_node("elem");
  
  for( ;NULL!=iter; iter=iter->next_sibling("elem") ) {
    rapidxml::xml_attribute<> *val = iter->first_attribute("value");
    
    if( NULL!=val ) {
      std::string txt(val->value(), val->value_size());

      addTextValue(txt);
    }
  }
}


// observers :

std::string BasicEnumerated::getStringValue(size_t i) const {
  std::ostringstream oss;
  print_value(oss, i);
  return oss.str();
}

std::ostream &BasicEnumerated::print_domain(std::ostream &out) const {
  size_t i, size = getSize();
  if( 1==size ) 
    return print_value(out, 0);
  else {
    print_value(out<<'{', 0);
    for( i=1; i<size; ++i ) 
      print_value(out<<", ", i);
    return out<<'}';
  }
}

std::ostream &BasicEnumerated::toXml(std::ostream &out, size_t tabs) const {
  std::ostream_iterator<char> pad(out);
  std::fill_n(pad, tabs, ' '); 
  size_t i, size = getSize();
  out<<'<'<<getTypeName();
  if( 0==size )
    return out<<"/>";
  else {
    out<<">\n";
    for( i=0 ; i<size; ++i ) {
      std::string strVal=getStringValue(i);
      
      std::fill_n(pad, tabs+1, ' '); 
      out<<"<elem value=\"";
      if( !strVal.empty() )
	// I know I use undocumented method but that's the easiest way to encode
	// special chars in a correct XML format
	rapidxml::internal::copy_and_expand_chars(strVal.c_str(), 
						  strVal.c_str()+strVal.length(),
						  '\0', 
						  std::ostream_iterator<char>(out));
      out<<"\"/>\n";
    }
    std::fill_n(pad, tabs, ' '); 
    return out<<"</"<<getTypeName()<<'>';
  }
}

// manipulators

void BasicEnumerated::accept(DomainVisitor &visitor) const {
  return visitor.visit(this);
}
