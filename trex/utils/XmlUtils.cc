/* -*- C++ -*- */
/** @file "XmlUtils.cc"
 * @brief XML utilities implmentation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */
/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2011, MBARI.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TREX Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
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
    // load the file and notify LogManager that this file was used
    name = s_log->use(name, found);
    if( !found ) 
      ErrnoExcept("Unable to locate "+name);
    ret.reset(new file<>(name.c_str()));
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
