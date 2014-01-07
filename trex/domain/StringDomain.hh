/* -*- C++ -*- */
/** @file "StringDomain.hh"
 * @brief string values enumeration domain
 *
 * This file defines the StringDamain class. 
 *
 * The StringDomain is a specialization in order to declare domains
 * based on string values. 
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup domains
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
#ifndef H_StringDomain
# define H_StringDomain

# include "EnumeratedDomain.hh"

namespace TREX {
  namespace transaction {

    /** @brief String domain
     *
     * This class implements a simple representation of strings domain.
     * The representation is based on an EnumeratedDomain that enumerates all
     * the possible values for this string.
     *
     * @author Frederic Py <fpy@mbari.org> 
     * @ingroup domains
     */
    class StringDomain 
      :public TREX::transaction::EnumeratedDomain<std::string> {
    public:
      static TREX::utils::symbol const type_name;
      /** @brief Default Constructor 
       * 
       * Creates a new full string domain.
       */
      StringDomain() 
	:TREX::transaction::EnumeratedDomain<std::string>(type_name) {}
      /** @brief Constructor
       * 
       * @tparam a C++ iterator type
       * @param from an iterator
       * @param to an iterator
       *
       * @pre [@e from,  @e to ) should be a valide iterator sequence
       * @pre @e Iter should points to std::string elements
       *
       * Creates a new instance restricted to all the elements in [@e from, @e to)
       *
       * @throw EmptyDomainthe created domain is empty
       */
      template<class Iter>
      StringDomain(Iter from, Iter to) 
	:TREX::transaction::EnumeratedDomain<std::string>(type_name, from, to) {}
      /** @brief Constructor
       * @param val a value
       *
       * Create a new domain with the single value @e val
       */
      StringDomain(std::string const &val) 
	:TREX::transaction::EnumeratedDomain<std::string>(type_name, val) {}
      /** @brief XML parsing constructor
       *
       * @param node an XML node
       *
       * Create a new domain based on the content of @e node. The type of
       * StringDomain is string.
       * @example
       * A full domain is defined by the empty element :
       * @code
       * <string/>
       * @endcode
       *
       * A string containing the two values "foo" and "bar bar" is described by
       * @code
       * <string>
       *   <elem value="foo"/>
       *   <elem value="bar bar"/>
       * </string>
       *@endcode
       */
      explicit StringDomain(boost::property_tree::ptree::value_type &node) 
	:TREX::transaction::EnumeratedDomain<std::string>(node) {}

      /** @brief Destructor */
      ~StringDomain() {}

      /** @brief Copy operator
       *
       * Allocates a new copy of current instance
       */
      DomainBase *copy() const {
	return new StringDomain(begin(), end());
      }
    }; // TREX::transaction::StringDomain


  } // TREX::transaction
} // TREX
      
      
#endif  // H_StringDomain
