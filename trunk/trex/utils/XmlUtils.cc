/* -*- C++ -*- */
/** @file "XmlUtils.cc"
 * @brief XML utilities implmentation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */
#include "XmlUtils.hh"

using namespace rapidxml;
using namespace TREX::utils;

/*
 * class TREX::utils::ext_iterator
 */ 

// statics :

SingletonUse<LogManager> ext_iterator::s_log;

ext_iterator::file_ref ext_iterator::load(xml_attribute<> *attr) {
  file_ref ret;
  
  if( NULL!=attr ) {
    std::string name(attr->value(), attr->value_size());
    bool found;
    name = s_log->use(name, found);
    if( !found ) 
      ErrnoExcept("Unable to locate "+name);
    // load the file and notify LogManager that this file was used
    ret.reset(new file<>(name.c_str()));
    // TODO add the use of LogManager
  }
  return ret;
}

// structors :

ext_iterator::ext_iterator(xml_node<> &node) 
  :m_external(false), m_current(node.first_node()),
   m_doc_root(NULL) {}

ext_iterator::ext_iterator(xml_node<> const &node, std::string const &conf) 
  :m_external(false), m_current(node.first_node()), 
   m_file(load(node.first_attribute(conf.c_str()))),
   m_doc_root(NULL) {
  if( m_file ) {
    m_doc.reset(new xml_document<>);
    m_doc->parse<0>(m_file->data());
    m_doc_root = m_doc->first_node();
    if( NULL==m_doc_root )
      throw XmlError(node, std::string("Attribute ")+conf+" refers to an empty file.");
    if( NULL==m_current ) {
      m_external = true;
      m_current = m_doc_root->first_node();
    }
  }
}

// manipulators :

ext_iterator ext_iterator::externalBegin(char *name) const {
  ext_iterator ret(*this);
  ret.m_external = true;
  if( ret.m_doc_root==NULL )
    ret.m_current = NULL;
  else
    ret.m_current = ret.m_doc_root->first_node(name);
  return ret;
}

ext_iterator::pointer ext_iterator::operator->() const {
  if( !valid() )
    throw Exception("Iterator referring to no node.");
  return m_current;
}

ext_iterator &ext_iterator::next(std::string const &name) {
  char const *name_c = NULL;
  if( !name.empty() )
    name_c = name.c_str();

  if( NULL!=m_current ) {
    m_current = m_current->next_sibling(name_c);
  } 
  if( NULL==m_current ) {
    if( !m_external && NULL!=m_doc_root ) {
      m_external = true;
      m_current = m_doc_root->first_node(name_c);
    }
  }
  return *this;
}

ext_iterator ext_iterator::find_tag(std::string const &name) const {
  if( NULL!=m_current && is_tag(*m_current, name) )
    return *this;
  else {
    ext_iterator tmp(*this);
    return tmp.next(name.c_str());
  }
}
