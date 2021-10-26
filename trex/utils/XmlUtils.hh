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
# include <boost/property_tree/ptree.hpp>
// Need to indicate to spirit to be thread safe
# define BOOST_SPIRIT_THREADSAFE
# include <boost/property_tree/xml_parser.hpp>

# include "StringExtract.hh"
# include "LogManager.hh"

namespace TREX {
  namespace utils {
    
    /** @brief XML parsing related errors.
     *
     * This exception is used for error related to XML parsing
     * and data extraction
     *
     * @todo As we migrate to Boost.PropertyTree the source fo the
     *   tree os not necessarily XML. Which means that I may need
     *   to rename this class.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    class XmlError :public Exception {
    public:
      explicit XmlError(std::string const &msg) throw()
      :Exception("XML error: "+msg) {}
      /** @brief Constructor
       * @param node A porperty tree node
       * @param msg A message
       *
       * Create a new exception associated to the XML node @a node
       * with the error message @a msg
       */
      XmlError(boost::property_tree::ptree::value_type const &node,
               std::string const &msg) throw()
      :Exception(std::string("On ")+
                 std::string(node.first)+": "+msg) {}
      /** @brief Destructor */
      virtual ~XmlError() throw() {}
    }; // TREX::utils::FactoryException
    
    namespace internals {
      
      /** @brief XML attribibute parsing helper
       *
       * @tparam Ty the expected output type
       *
       * This class is used internally as an helper to extract the value of an
       * XML attribute and parse it as a @p Ty instance
       *
       * It is used by the parse_attr methods in order to extract and parse an
       * attribute from an XML tag.
       *
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup utils
       */
      template<class Ty, bool AttrOptional=true>
      struct attr_helper {
        
        static std::string get_str(boost::property_tree::ptree const &pt,
                                   std::string const &name) {
          std::string path("<xmlattr>."+name);
          if constexpr (AttrOptional) {
            if(auto ret=pt.get_optional<std::string>(path); ret) {
              return *ret;
            }
            path = name;
          } 
          return pt.get<std::string>(path);
        }
        
        /** @brief Parse attribute
         *
         * @param[in] pt A xml based property tree
         * @param[in] name An attribute name
         *
         * Extracts the value of the attribute @p name from the property tree
         * @p pt
         *
         * @pre The path @c "<xmlattr>."+@p name exists or @p Ty is a std::optional
         * @pre The value of the attribute can be parsed as a @p Ty
         *
         * @return the value of this attribute
         * @note if @p Ty is a std::optional and the attribute @p name does
         *       not exists the returned value is an empty optional
         *
         * @throw bad_string_cast Failed to convert the attribute
         *     @p name into a @p Ty value
         */
        static Ty get(boost::property_tree::ptree const &pt,
                      std::string const &name) {
          return string_cast<Ty>(get_str(pt, name));
        }
      }; // TREX::utils::internals::attr_helper<>
      
#ifndef DOXYGEN
      template<class Ty, bool AttrOptional>
      struct attr_helper< std::optional<Ty>, AttrOptional > {
        
        static std::optional<std::string> get_str(boost::property_tree::ptree const &pt,
                                                    std::string const &name) {
          std::string path("<xmlattr>."+name);
          
          if constexpr ( AttrOptional ) {
            if(auto ret=pt.get_optional<std::string>(path); ret)
              return *ret;
            path = name;
          }
          if(auto ret = pt.get_optional<std::string>(path); ret)  {
            return *ret;
          }
          return std::nullopt;
        }
        
        static std::optional<Ty> get(boost::property_tree::ptree const &pt,
                                       std::string const &name) {
          std::optional<std::string> tmp = get_str(pt, name);
          if( tmp )
            return std::optional<Ty>(string_cast<Ty>(*tmp));
          else
            return std::optional<Ty>();
        }
      }; // TREX::utils::internals::attr_helper< std::optional<> >
#endif // DOXYGEN
      
    } // TREX::utils::internals
    
    /** @brief XML attribute extraction
     * @tparam Ty output type
     * @param node An XML node
     * @param attr Attribute name
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
     * @author Frederic Py
     * @{
     */
    template<class Ty>
    Ty parse_attr(boost::property_tree::ptree const &node,
                  std::string const &attr) {
      try {
        return internals::attr_helper<Ty>::get(node, attr);
      } catch(boost::property_tree::ptree_bad_data const &e) {
        throw XmlError("Failed to parse attribute \""+attr+"\": "+e.what());
      }
    }
    
    template<class Ty>
    Ty parse_attr(boost::property_tree::ptree::value_type const &node,
                  std::string const &attr) {
      try {
        return parse_attr<Ty>(node.second, attr);
      } catch(XmlError const &n) {
        throw XmlError(node, n.what());
      }
    }
    /** @} */
    
    /** @brief XML attribute insertion
     * @tparam Ty value type
     * @param[in,out] node An XML node
     * @param[in] attr Attribute name
     * @param[in] value A value
     *
     * Add the attribute @p attr to the xml node @p node with the value @p value
     *
     * @pre Ty is a type that supports standard output streams
     *
     * @ingroup utils
     * @author Frederic Py
     * @{
     */
    template<typename Ty>
    void set_attr(boost::property_tree::ptree &node,
                  std::string const &attr, Ty const &value) {
      node.put("<xmlattr>."+attr, value);
    }
    
    template<typename Ty>
    void set_attr(boost::property_tree::ptree::value_type &node,
                  std::string const &attr, Ty const &value) {
      set_attr(node.second, attr, value);
    }
    /** @} */
    
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
     * @author Frederic Py
     * @sa parse_attr(boost::property_tree::ptree const &, std::string const &)
     * @{
     */
    template<class Ty>
    Ty parse_attr(Ty const &on_missing,
                  boost::property_tree::ptree const &node,
                  std::string const &attr) {
      std::optional<Ty> ret = parse_attr< std::optional<Ty> >(node, attr);
      if( ret )
        return *ret;
      return on_missing;
    }
    
    template<class Ty>
    Ty parse_attr(Ty const &on_missing,
                  boost::property_tree::ptree::value_type const &node,
                  std::string const &attr) {
      return parse_attr<Ty>(on_missing, node.second, attr);
    }
    /** @} */
    
    
    /** brief Check for XML tag name
     * @param node A property tree value
     * @param tag A tag name
     * @retval true if the tag name of @a node is @a tag
     * @retval false otherwise
     * @ingroup utils
     */
    inline bool is_tag(boost::property_tree::ptree::value_type const &node,
                       std::string const &tag) {
      return tag==node.first;
    }
    
    /** @brief Extend XML tree with extra file
     *
     * @param[in, out] tree A XML tree
     * @param[in] conf External configuration attribute
     * @param[in] ahead Position flag
     *
     * Add tha content of the fille referred by the attribute @p conf to @p tree.
     *
     * This methods parse the XML file referred by the attribute @p vconf of @p tree
     * and add its content as sub tags of @p tree. If @p ahead is @c true this content
     * will be added before the @p tree sub tags, otherwise it will be added after them.
     *
     * @note If @p conf is not an attribute of @p tree then @p tree is not modified
     *
     * @pre if @p conf is an attribute of @p tree its value should be the name of a file
     *   that can be located by LogManager and is a valid XML file
     *
     * @throw XmlError the @p conf file is not a valid XML file or is empty
     * @throw ErrnoExcept Failed to locate the @p conf file
     *
     * @post If @p conf was an attribute of @p tree then @p tree hass been modified to include the content of
     * @p conf file
     *
     * Illustration.
     *
     * Assume that we have a @p tree with this XML content :
     * @code
     * <Root file="foo.xml">
     *   <Son/>
     * </Root>
     * @endcode
     *
     * And the file @c foo.xml has the following content:
     * @code
     *  <Daughter/>
     *  <Cousin/>
     * @endcode
     *
     * By calling @c ext_xml(tree,"file") we will modify @p tree as follow:
     * @code
     * <Root file="foo.xml">
     *   <Daughter/>
     *   <Cousin/>
     *   <Son/>
     * </Root>
     * @endcode
     *
     * @author Frederic Py
     * @ingroup utils
     * @sa LogManager
     */ 
    void ext_xml(boost::property_tree::ptree &tree,
                 std::string const &conf, bool ahead=true);
    
  } // TREX::utils
} // TREX

#endif // H_XmlUtils
