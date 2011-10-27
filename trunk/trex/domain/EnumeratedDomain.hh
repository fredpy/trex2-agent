/* -*- C++ -*- */
/** @file "EnumeratedDomain.hh"
 * @brief enumeration based domain definition
 *
 * This file defines the basic template class used to define enumeration
 * based domains.
 *
 * An enumeration based domain is a non continuous finite domain that can
 * be described as a set of possible values. Classical example are C/C++
 * enums or even strings (even though it is not finite this is generally
 * a ressonnable approximation)
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
#ifndef H_EnumeratedDomain 
# define H_EnumeratedDomain

# include <algorithm>
# include <iterator>
# include <set>

# include "BasicEnumerated.hh"

namespace TREX {
  namespace transaction {
    
    /** @brief Enumeration based domain
     *
     * @param Ty type of the elements
     * @param Comp Ordering function
     *
     * This template class provides a basis for defining enumeration
     * based domains.
     *
     * An enumerated domain is a domain that  can easily be represented as
     * a set of possible values. 
     * 
     * @pre @e Comp is expected to be a complete order over @e Ty
     *
     * @note As for now this domain relies on the set standard C++ compiler.
     * This results on the need to have the elements being comparable though
     * an ordering functor. We may in the future use the more general
     * collection_set, provided by next C++ standard (and already available
     * on the boost library),
     * which will remove this requirement.
     *
     * @note To ease implementation of this class the full domain is
     * represented by the fact that the subjacent value collection is empty.
     * This simplifies the implementation of the default dcontructor of this
     * domain which is implicitely expected to generate a full domain.
     * If one wnats to derive from this class to create domains with fully
     * specified set of possible values, he will also preferrably have
     * to redefine the isFull method to reflect this.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup domains
     */
    template< typename Ty, class Comp=std::less<Ty> >
    class EnumeratedDomain :public BasicEnumerated {
    private:
      /** @brief Internal domain elements container */
      typedef std::set<Ty, Comp> container_type;
      
    public:
      /** @brief Possible values iterator */
      typedef typename container_type::const_iterator iterator;
      
      /** @brief Constructor
       *
       * @param type a symbolic type name
       *
       * Create a new instance as a full domain of type @e type.
       */
      explicit EnumeratedDomain(TREX::utils::Symbol const &type)
      :BasicEnumerated(type) {}
      /** @brief Constructor
       * @tparam Iter an iterator type
       * @param type A symbolic type name
       * @param from An iterator
       * @param to An iterator
       *
       * Create a new instance of type @e type containing all the elements
       * between @e from and @e to
       *
       * @pre [from, to) should be a valid iterator sequence
       * @pre [from to) is not empty (ie from!=to)
       * @pre the type referred by @e Iter should be convertible into @e Ty
       * @throw EmptyDomain the created domain is empty
       */
      template<class Iter>
      EnumeratedDomain(TREX::utils::Symbol const &type,
                       Iter from, Iter to)
      :BasicEnumerated(type), m_elements(from, to) {
        if( m_elements.empty() )
          throw EmptyDomain(*this, "Newly created EnumeratedDomain is empty.");
      }
      
      /** @brief Constructor
       * @param type A symbolic type name
       * @param val A value
       *
       * Create a new instance of type @e type containing only the element
       * @e val
       */
      explicit EnumeratedDomain(TREX::utils::Symbol const &type,
                                Ty const &val)
      :BasicEnumerated(type) {
        add(val);
      }
      
      /** @brief Destructor */
      virtual ~EnumeratedDomain() {}
      
      // Extra helpers
      /** @brief Add an element to the domain
       *
       * @param val A value
       *
       * This method allows user to add @e val into the possible values
       * of this domain.
       *
       * @note this method is an helper to ease the construction of
       * enumerated domains. It should be used only when the domain is
       * constructed and still not public to other reactors. Overwise
       * using it represent a domain relaxation which could be problematic
       * for third party components such as planner based reactors.
       *
       * @sa template<class Iter> void add(Iter, Iter)
       */
      void add(Ty const &val) {
        m_elements.insert(val);
      }
      /** @brief Add an element to the domain
       *
       * @tparam Iter An iterator type 
       * @param from an iterator
       * @param to an iterator
       *
       * This method allows user to add all the elements referred by [from, to)
       * into the possible values of this domain.
       *
       * @pre [from, to) should be a valid iterator sequence
       * @pre the type referred by @e Iter should be convertible into @e Ty
       *
       * @note this method is an helper to ease the construction of
       * enumerated domains. It should be used only when the domain is
       * constructed and still not public to other reactors. Overwise it
       * using it represent a domain relaxation which could be problematic
       * for third party components such as planner based reactors.
       *
       * @sa void add(Ty const &)
       */
      template<class Iter>
      void add(Iter from, Iter to) {
        m_elements.insert(from, to);
      }
      
      /** @brief test for membership
       * @param val A value
       * @retval true if @e val is a possible value for this domain
       * @retval false otherwise
       */
      bool contains(Ty const &val) const {
        return isFull() || end()!=m_elements.find(val);
      }
      bool intersect(DomainBase const &other) const;
      bool equals(DomainBase const &other) const;
      DomainBase &restrictWith(DomainBase const &other);
      
      size_t getSize() const {
        return m_elements.size();
      }
      
      boost::any getElement(size_t i) const {
        typename container_type::const_iterator it = begin();
        // std::set doe not provide operator[]
        // I need to do the iteration by my own
        for(; i>0 && end()!=it; ++it, --i);
        // if it==end() the behavior is undefined
        // but that also means that i>=getSize()
        return *it;
      }
      /** @brief First element
       *
       * This method allows user to iterate through the possible values
       * of the domain using classic C++ iterator paradigm.
       *
       * @return An iterator to the beginnig of the possible values collection
       */
      iterator begin() const {
        return m_elements.begin();
      }
      /** @brief End iterator
       *
       * This method allows user to iterate through the possible values
       * of the domain using classic C++ iterator paradigm.
       *
       * @return An iterator to the end of the possible values collection
       */
      iterator end() const {
        return m_elements.end();
      }
      
    protected:
      /** @brief Copy constructor
       *
       * @param other Another instance
       *
       * Create a new instance which is a copy of @e other
       */
      EnumeratedDomain(EnumeratedDomain const &other)
      :BasicEnumerated(other), m_elements(other.m_elements) {}
      /** @brief XML parsing constructor
       * @param node An XML node
       *
       * Create a new instance by parsing the content of @e node.
       * The expected structure of this node is expected to be of
       * a from similar to the following :
       * @code
       * < <type> >
       *   <elem value="<val1>"/>
       *   <elem value="<val2>"/>
       *   [...]
       * </ <type> >
       * @endcode
       * where @c @<type@> is the type of the domain
       * and @c @<vali@> are possible values for this domain
       * @throw TREX::utils::bad_string_cast Unable to parse one of
       * the @c @<vali@> attributes
       *
       * @note If no elem tag is defined into @e node, the domain
       * will be considered as full.
       */
      EnumeratedDomain(boost::property_tree::ptree::value_type &node) 
      :BasicEnumerated(node) {
        completeParsing(node);
      }
      
      void addTextValue(std::string const &val) {
        add(TREX::utils::string_cast<Ty>(val));
      }
      std::ostream &print_value(std::ostream &out, size_t i) const {
        // Not very optimal but this is the easiest way to go
        return out<<(getTypedElement<Ty, false>(i));
      }
      
    private:
      /** @brief Domain elements
       *
       * This attribute stores the possible values of this domain.
       *
       * @note for ease of implementation a domain is full when this
       * collection is empty. It is counter intuitive but simplifies
       * the implementation of the default constructor which is
       * expected to create a full domain
       */
      container_type m_elements;
      
    }; // EnumeratedDomain<>
    
    /*
     * class EnumeratedDomain<>
     */
    
    template<class Ty, class Cmp>
    bool EnumeratedDomain<Ty, Cmp>::intersect(DomainBase const &other) const {
      if( getTypeName()!=other.getTypeName() ) 
        return false;
      else {
        EnumeratedDomain<Ty, Cmp> const &ref 
        = dynamic_cast<EnumeratedDomain<Ty, Cmp> const &>(other);
        if( isFull() || ref.isFull() )
          return true;
        else {
          iterator i = begin(), j = ref.begin();
          iterator const endi = end();
          iterator const endj = ref.end();
          
          while( endi!=i && endj!=j ) {
            if( m_elements.key_comp()(*i, *j) )
              ++i;
            else if( m_elements.key_comp()(*j, *i) )
              ++j;
            else 
              // *i and *j are equals -> intersection is not empty
              return true;
          }
          // no similar items -> itersection is empty
          return false;
        }
      }
    }
    
    template<class Ty, class Cmp>
    bool EnumeratedDomain<Ty, Cmp>::equals(DomainBase const &other) const {
      if( getTypeName()!=other.getTypeName() ) 
        return false;
      else {
        EnumeratedDomain<Ty, Cmp> const &ref 
        = dynamic_cast<EnumeratedDomain<Ty, Cmp> const &>(other);
        return m_elements==ref.m_elements;
      }
    }
    
    template<class Ty, class Cmp>
    DomainBase &EnumeratedDomain<Ty, Cmp>::restrictWith(DomainBase const &other) {
      if( getTypeName()!=other.getTypeName() ) 
        throw EmptyDomain(*this, "Incompatible types");
      else {
        EnumeratedDomain<Ty, Cmp> const &ref 
        = dynamic_cast<EnumeratedDomain<Ty, Cmp> const &>(other);    
        container_type tmp;
        std::set_intersection(begin(), end(), ref.begin(), ref.end(), 
                              std::inserter(tmp, tmp.begin()), 
                              m_elements.key_comp());
        if( tmp.empty() ) 
          throw EmptyDomain(*this, "intersection is empty.");
        m_elements.swap(tmp);
      }
      return *this;
    }
    
    
  } // TREX::transaction
} // TREX

#endif // H_EnumeratedDomain
