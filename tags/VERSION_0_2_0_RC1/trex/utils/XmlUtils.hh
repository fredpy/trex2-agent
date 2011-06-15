/* -*- C++ -*- */
/** @file "XmlUtils.hh"
 * @brief Helpers for xml manipulation
 * 
 * This file defines a set of utilisties that eas manipulation of
 * rapidxml structures and data extraction
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
#ifndef H_XmlUtils 
# define H_XmlUtils

# include <boost/shared_ptr.hpp>

# include <rapidxml/rapidxml_utils.hpp>
# include "StringExtract.hh"
# include "Exception.hh"
# include "LogManager.hh"

namespace TREX {
  namespace utils {

    /** @brief XML parsing related errors.
     *
     * This exception is used for error related to XML parsing
     * and data extraction
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    class XmlError :public Exception {
    public:
      /** @brief Constructor
       * @param node An XML node
       * @param msg A message
       *
       * Create a new exception associated to the XML node @a node
       * with the error message @a msg
       */
      XmlError(rapidxml::xml_node<> const &node,
	       std::string const &msg) throw() 
	:Exception(std::string("On ")+
		   std::string(node.name(), 
			       node.name_size())+": "+msg) {}
      /** @brief Destructor */
      virtual ~XmlError() throw() {}
    }; // TREX::utils::FactoryException
    
    /** @brief XML attribute extraction
     * @tparam Ty output type
     * @param node An XML node
     * @param attr Attribyute name
     *
     * This method extract attribute @a attr from @a node and parses it
     * as a @a Ty variable
     *
     * @pre @a node has the attribute @a attr
     * @pre the value of @a attr can be parsed into the type @a Ty
     * @throw XmlError Unable to find attribute @a attr
     * @throw bad_string_cast Cannot parse the value of @a attr as @a Ty
     *
     * @return The parsed value from @a attr
     * @sa template<class Ty> Ty const &parse_attr(Ty const &, rapidxml::xml_node<> const &, std::string const &attr)
     * @ingroup utils
     */
    template<class Ty>
    Ty parse_attr(rapidxml::xml_node<> const &node, 
		  std::string const &attr) {
      rapidxml::xml_attribute<> const *a = node.first_attribute(attr.c_str());
      if( NULL==a ) 
	throw XmlError(node, std::string("Unable to find attribute ")+attr);
      return string_cast<Ty>(std::string(a->value(), a->value_size()));
    }

    /** @brief Optional XML attribute extraction
     * @tparam Ty output type
     * @param on_missing default value if missing
     * @param node An XML node
     * @param attr Attribyute name
     *
     * This method extract attribute @a attr from @a node and parses it
     * as a @a Ty variable. If the attribute is missing it returns the
     * default @a on_missing value
     *
     * @pre the value of @a attr can be parsed into the type @a Ty
     * @throw bad_string_cast Cannot parse the value of @a attr as @a Ty
     *
     * @retval @a on_missing If @a attr does not exist in @a node
     * @return The parsed value from @a attr otherwise
     * 
     * @sa template<class Ty> Ty const &parse_attr(rapidxml::xml_node<> const &, std::string const &attr)
     * @ingroup utils
     */
    template<class Ty>
    Ty parse_attr(Ty const &on_missing, 
		  rapidxml::xml_node<> const &node, 
		  std::string const &attr) {
      rapidxml::xml_attribute<> const *a = node.first_attribute(attr.c_str());
      if( NULL==a )
	return on_missing;
      else 
	return string_cast<Ty>(std::string(a->value(), a->value_size()));
    }

    /** brief Check for XML tag name
     * @param node A XML node
     * @param tag A tag name
     * @retval true itf the tag name of @a node is @a tag
     * @retval false otherwise
     * @ingroup utils
     */
    inline bool is_tag(rapidxml::xml_node<> const &node,
		       std::string const &tag) {
      return 0==tag.compare(0, std::string::npos, 
			    node.name(), node.name_size());
    }

    /** @brief XML nodes iterator through files
     *
     * This class implements an iterator for the childs of a node
     * that can optional load an extranl file provide by an attribute
     * of this node to provide extra tags
     *
     * This allow to easily provide configuration externalization
     * in optional files.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    class ext_iterator {
    public:
      /** @brief Referred objects type 
       */
      typedef rapidxml::xml_node<>  value_type;
      /** @brief Type of a reference to the referred object
       */
      typedef rapidxml::xml_node<> &reference;
      /** @brief Pointer type to the referred object
       */
      typedef rapidxml::xml_node<> *pointer;

      /** @brief default constructor
       * Creates an iterator pointing to nothing
       */
      ext_iterator()
	:m_current(NULL), m_doc_root(NULL) {}
      /** @brief Copy constructor
       * @param other
       *
       * Creates a copy of @a other
       */
      ext_iterator(ext_iterator const &other)
	:m_external(other.m_external), m_current(other.m_current),
	 m_file(other.m_file), m_doc(other.m_doc), 
	 m_doc_root(other.m_doc_root) {}
      /** @brief Simple child iterator
       * @param node An XML node
       *
       * Creates an iterator for the child nodes of @a node
       */
      explicit ext_iterator(rapidxml::xml_node<> &node);
      /** @brief Child and external config iterator
       * @param node An XML node
       * @param conf An optional attribute of @a node
       *
       * Creates an iterator that allows to iterate on both the childs of
       * @a node and on the content of the file referred by the attribute
       * @a conf.
       * If @a conf attribute does not exist the instance will only
       * iterate through the childs.
       * If @a conf exist it will also iterate thought the childs of the
       * root node fo the referred file.
       *
       * @pre if @a conf attribute exist it must be an existing file
       * @pre this file should be a valid and non empty XML file
       *
       * @throw std::runtime_error Failed to open the file
       * @throw rapidxml::parse_error Unable to parse the file
       * @throw XmlError The file is empty
       * @throw Exception file not found
       */ 
      ext_iterator(rapidxml::xml_node<> const &node, std::string const &conf);
      /** @brief Destructor */
      ~ext_iterator() {}

      /** @brief Check for validity
       *
       * @retval true if current instance refer to a node
       * @retval false else
       *
       * @sa bool operator !() const
       */
      bool valid() const {
	return NULL!=m_current;
      }
      /** @brief Check for validity
       *
       * @return !valid()
       *
       * @sa bool valid() const
       */
      bool operator !() const {
	return !valid();
      }
      /** @brief Check for external node
       *
       * This method helps user to identify whether the referred node is
       * a direct child of the node used for initial construction or if
       * it is coming from the file loaded with an optional tag
       * 
       * @retval true if the referred node is coming from the
       * file loaded with external tag
       * @retval false otherwise
       *
       * @note As soon as this value becomes @c true all valid successor will
       * be external too.
       * @note if @c hasExternal() is @c false then this method will
       * never return @c true
       *
       * @sa bool hasExternal() const
       */
      bool external() const {
	return valid() && m_external;
      }
      /** @brief Check for external file
       *
       * This methods indicate whether an external file was loaded
       * for this iterator or not
       *
       * @retval true if an external file was loaded
       * @retval false else
       */
      bool hasExternal() const {
	return NULL!=m_doc_root;
      }

      /** @brief First external node
       * @param name A tag name
       *
       * This method creates a node pointing
       * to the first node with tag @a name
       * that is external
       *
       * @return the iterator pointing to firts external node
       * or an invalid node if this iterator has no external node
       *
       * @note if @ name is  NULL or not given this method just
       * look for the first external node
       *
       * @sa bool hasExternal() const
       */
      ext_iterator externalBegin(char *name=NULL) const;
	
      /** @brief access operator
       *
       * Give access to the referred node attributes and methods
       * @pre current instance should be valid
       * @throw NullAccess this instance is not valid
       *
       * @sa bool valid() const
       * @sa reference operator *() const
       */
      pointer   operator->() const;
      /** @brief access operator
       *
       * Give access to the referred node 
       * @pre current instance should be valid
       * @throw NullAccess this instance is not valid
       *
       * @sa bool valid() const
       * @sa pointer operator->() const
       */
      reference operator* () const {
	return *operator->();
      }
      
      /** @brief advance to next node
       * @param name tag of the node
       *
       * This method advance to the next node. If @a name is specified
       * and not @c NULL, the search is restricted to the nodes with
       * the tag @a name
       *
       * @return the instance after the operation
       *
       * @sa ext_iterator &operator++()
       */
      ext_iterator &next(std::string const &name =std::string());
      /** @brief advance to next node
       * 
       * This method advance to the next node.
       *
       * @return the instance after the operation
       *
       * @sa ext_iterator &next(char *name)
       */
      ext_iterator &operator++() {
	return next();
      }
      
      /** @brief Find element
       * @param name a tag name
       *
       * This method locates the iterator pointing to element with
       * the tag @a name. If current instance is match es the tag then
       * it is returned imediately as oopp[opsed to the behavior of @c next
       *
       * @sa ext_iterator &next(char *name)
       */
      ext_iterator find_tag(std::string const &name) const;

    private:
      bool m_external;
      rapidxml::xml_node<> *m_current;
      
      typedef boost::shared_ptr< rapidxml::file<> > file_ref;
      typedef boost::shared_ptr< rapidxml::xml_document<> > doc_ref;
      file_ref m_file;
      doc_ref m_doc;
      rapidxml::xml_node<> *m_doc_root;

      /** @brief File load
       * @param attr Pointer to an attribute
       *
       * Load in memory the file described by the value of @a attr
       * if @a attr is not a @c NULL pointer.
       *
       * @pre if @attr is not @a NULL its value has to refer to an
       * existing file
       *
       * @throw Exception file not found
       *
       * @return A pointer which is either null or referring to
       * the file content
       */
      static file_ref load(rapidxml::xml_attribute<> *attr);

      static SingletonUse<LogManager> s_log;
    }; 

    template<class Ty>
    Ty xml_conf_var(ext_iterator const &conf, std::string const &tag) {
      ext_iterator tmp(conf.find_tag(tag));
      if( !tmp )
	throw XmlError(*conf, "Unable to find tag \""+tag+"\".");
      return parse_attr<Ty>(*tmp, "value");
    }

    template<class Ty>
    Ty xml_conf_var(ext_iterator const &conf, std::string const &tag, 
		    Ty const &on_missing) {
      ext_iterator tmp(conf.find_tag(tag));
      if( !tmp )
	return on_missing;
      return parse_attr<Ty>(on_missing, *tmp, "value");
    }
    
  } // TREX::utils
} // TREX

#endif // H_XmlUtils
