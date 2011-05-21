/** @file "Predicate.cc"
 * @brief Precicate class implementation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup transaction
 */  
#include "Predicate.hh"

using namespace TREX::utils;
using namespace TREX::transaction;

/*
 * class TREX::transaction::Predicate
 */ 
// structors :

Predicate::Predicate(rapidxml::xml_node<> const &node)
  :m_object(parse_attr<Symbol>(node, "on")),
   m_type(parse_attr<Symbol>(node, "pred")) {
  if( m_object.empty() ) 
    throw PredicateException("Empty \"on\" attribute in XML tag");
  if( m_type.empty() )
    throw PredicateException("Empty \"pred\" attribute in XML tag");
  rapidxml::xml_node<> *iter = node.first_node("Variable");
  for(; NULL!=iter; iter = iter->next_sibling("Variable") ) {
    Variable var(*iter);
    restrictAttribute(var);
  }
}

Predicate::Predicate(Predicate const &other)
  :m_object(other.m_object), m_type(other.m_type), m_vars(other.m_vars) {}

Predicate::~Predicate() {}


// modifiers :

void Predicate::restrictAttribute(Variable const &var) {
  if( !var.isComplete() )
    throw PredicateException("Predicate attribute is not fully defined");
  iterator pos = m_vars.lower_bound(var.name());
  if( end()!=pos && var.name()==pos->second.name() ) {
    pos->second.restrict(var);
  } else {
    // probably need to check if the varaible is valid/OK
    m_vars.insert(pos, std::make_pair(var.name(), var));
  }
}

// observers :

Variable const &Predicate::getAttribute(Symbol const &name) const {
  const_iterator pos = m_vars.find(name);
  if( end()==pos ) 
    throw PredicateException("Attribute \""+name.str()+"\" is unknown");
  return pos->second;
}

std::ostream &Predicate::print_to(std::ostream &out) const {
  std::list<Symbol> vars;
  out<<m_object<<'.'<<m_type;
  listAttributes(vars, false);
  if( !vars.empty() ) {
    out<<'{'<<getAttribute(vars.front());
    for( vars.pop_front(); !vars.empty(); vars.pop_front() )
      out<<", "<<getAttribute(vars.front());
    out<<'}';
  }
  return out;
}

std::ostream &Predicate::toXml(std::ostream &out, size_t tabs) const {
  std::list<Symbol> vars;
  std::ostream_iterator<char> pad(out);
  
  std::fill_n(pad, tabs, ' ');
  out<<"<"<<getPredTag()<<" on=\""<<object()
     <<"\" pred=\""<<predicate()<<'\"';
  listAttributes(vars, false);
  if( vars.empty() )
    out<<"/>";
  else {
    out<<">\n";
    do {
      getAttribute(vars.front()).toXml(out, tabs+1)<<'\n';
      vars.pop_front();
    } while( !vars.empty() );
    std::fill_n(pad, tabs, ' ');
    out<<"</"<<getPredTag()<<'>';
  }
  return out;
}

void Predicate::listAttributes(std::list<TREX::utils::Symbol> &attrs,
			       bool all) const {
  const_iterator i = begin();
  const_iterator const endi = end();
  for( ; endi!=i; ++i )
    if( i->second.isComplete() &&
	( all || !i->second.domain().isFull() ) )
      attrs.push_back(i->first);
}
