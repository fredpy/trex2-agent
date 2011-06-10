/** @file "Variable.cc"
 * @brief Variable class implementation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup domains
 */
# include <algorithm>
# include <iterator>

# include "Variable.hh"

using namespace TREX::utils;
using namespace TREX::transaction;

/*
 * class TREX::transaction::Variable
 */

// statics :

SingletonUse<DomainBase::xml_factory> Variable::s_dom_factory;

DomainBase *Variable::clone
(boost::call_traits<Variable::domain_ptr>::param_type dom) {
  if( !dom )
    return NULL;
  else
    return dom->copy();
}

// structors :

Variable::Variable() {}

Variable::Variable(Symbol const &name, DomainBase *dom)
  :m_name(name), m_domain(dom) {}

Variable::Variable(Symbol const &name, DomainBase const &dom) 
  :m_name(name), m_domain(dom.copy()) {
  if( m_name.empty() )
    throw VariableException("Empty variable names are not allowed");
}

Variable::Variable(Variable const &other) 
  :m_name(other.m_name), m_domain(clone(other.m_domain)) {}

Variable::Variable(rapidxml::xml_node<> const &node)
  :m_name(parse_attr<Symbol>(node, "name")) {
  if( m_name.empty() )
    throw XmlError(node, "Variable name is empty.");
  rapidxml::xml_node<> *dom_tag = node.first_node();
  if( NULL==dom_tag ) {
    throw XmlError(node, "Missing variable domain on XML tag"); 
  } else {
    m_domain = s_dom_factory->produce(*dom_tag);
  }
}

Variable::~Variable() {}

// Modifiers :

Variable &Variable::operator= (Variable const &other) {
  m_name = other.m_name;
  m_domain.reset(clone(other.m_domain));
  return *this;
}

Variable &Variable::restrict(DomainBase const &dom) {
  if( !m_domain )
    m_domain.reset(dom.copy());
  else 
    m_domain->restrictWith(dom);
  return *this;
}

Variable &Variable::restrict(Variable const &var) {
  if( var.m_name.empty() ) {
    if( var.m_domain )
      restrict(*(var.m_domain));
  } else if( m_name.empty() ) {
    if( var.m_domain )
      restrict(*(var.m_domain));
    m_name = var.m_name;
  } else {
    if( m_name!=var.m_name ) {
      throw VariableException("Cannot merge variables with different names.");
    }
    if( var.m_domain )
      restrict(*(var.m_domain));
  }
  return *this;
}

// Observers :

DomainBase const &Variable::domain() const {
  if( !m_domain ) 
    throw VariableException("Variable's domain cannot be accessed.");
  return *m_domain;
} 

std::ostream &Variable::print_to(std::ostream &out) const {
  if( isComplete() ) 
    return out<<m_name<<"="<<*m_domain;
  else
    return out<<"<?"<<m_name<<'>'; 
}

std::ostream &Variable::toXml(std::ostream &out, size_t tabs) const {
  std::ostream_iterator<char> pad(out);
  std::fill_n(pad, tabs, ' ');
  out<<"<Variable name=\""<<name()<<"\"";
  if( !m_domain ) 
    out<<"/>";
  else {
    out<<">\n";
    m_domain->toXml(out, tabs+1)<<'\n';
    std::fill_n(pad, tabs, ' ');
    out<<"</Variable>";
  } 
  return out;
}

