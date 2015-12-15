/* -*- C++ -*- */
/** @file "BasicEnumerated.hh"
 * @brief Abstract interface for enumerated domains
 *
 * This file defines the bases class for implementing enumerated
 * domains in TREX. It offers multiple utilities to have unified
 * access/input/output for this kind of domain.
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
#ifndef H_BasicEnumerated
# define H_BasicEnumerated

# include "abstract_domain.hh"

namespace trex {
  namespace transaction {
    
    /** @brief Enumerated Domain base class
     *
     * This class provide a unified abstract interface for implementing
     * enumerated domains. It especially provide a way to ease and unify
     * the access methods and input/output for all the domain of this kind.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup domains
     */
    class basic_enumerated :public abstract_domain {
    public:
      /** @brief Destructor */
      virtual ~basic_enumerated() {}
      
      bool is_interval() const {
        return false;
      }
      bool is_enumerated() const {
        return true;
      }
      
      bool is_full() const {
        return 0==size();
      }
      bool is_singleton() const {
        return 1==size();
      }
      
      /** @brief element count
       *
       * This method gives access to the number of elements in this instance.
       * @return the number of values associated to this domain
       *
       * @note by convention if the returned value is 0, TREX will
       * consider this domain as full
       */
      virtual size_t size() const =0;
      /** @brief generic element access
       * @param i index of the element
       *
       * This method give access to the element @a i of the domain.
       *
       * @pre @a i is less than the size of the set
       *
       * @return the element at index @a i encapsulated on the
       * @c boost::any class
       *
       * @sa size_t getSize() const
       */
      virtual boost::any element(size_t i) const =0;
      /** @overload boost::any getElement(size_t i) const
       */
      boost::any operator[](size_t i) const {
        return element(i);
      }
      /** @brief Typed element access
       * @tparam Ty   expected return type
       * @tparam Safe extraction method
       * @param i index of the element
       *
       * This method give access to the element @a i of the domain converted
       * in the type @a Ty. The @a Safe flag indicate whether the conversion to
       * @a Ty should be done using @c boost::any_cast -- which will fail if
       * the encapsulated type is not @a Ty -- or using the safer string_cast
       * that should be more type robust
       *
       * @pre @a i is less than the size of the set
       * @pre if @a Safe is false, the underlying type of the container has
       * to be @u exactly @a Ty
       * @pre if @a Safe is true the string value of this element should be
       * parseable into @a Ty
       *
       * @return the element at index @a i converted to the type @a Ty
       *
       * @note No matter what this method is heavy in term of computation
       * overhead and should be used wisely (mostly for debugging/instrumentation
       * purpose)
       *
       * @sa size_t getSize() const
       * @sa boost::any getElement(size_t i) const
       * @sa std::string getStringValue(size_t i) const
       */
      template< class Ty, bool Safe >
      Ty typed_element(size_t n) const {
        boost::any val = element(n);
        if( Safe ) {
          Ty *ret = boost::any_cast<Ty>(&val);
          if( NULL!=ret )
            return *ret;
          else
            return boost::lexical_cast<Ty>(element_as_string(n));
        } else
          return boost::any_cast<Ty>(val);
      }
      
      /** @brief Get element textual value
       * @param[in] i Index of the element
       *
       * Get the value of the element @p i of the domain in its textual form
       *
       * @pre @p i is a valid index (between 0 and the size of the set-1)
       *
       * @return A string with a human readable for the value of the element @p i
       *
       * @sa getSize() const
       * @sa getElement(size_t) const
       * @sa getTypedElement(size_t) const
       */
      virtual std::string element_as_string(size_t i) const;
      
      virtual void add_string_value(std::string const &val) =0;
    protected:
      explicit basic_enumerated(utils::symbol const &type)
      :abstract_domain(type) {}
      explicit basic_enumerated(boost::property_tree::ptree::value_type &node)
      :abstract_domain(node) {}
      
      void complete_parsing(boost::property_tree::ptree::value_type &node);
      
      virtual std::ostream &print_value(std::ostream &out, size_t i) const =0;
      
    private:
      boost::property_tree::ptree build_tree() const;
      
      void accept(domain_visitor &visitor) const;
      std::ostream &print_domain(std::ostream &out) const;
      
      boost::any singleton() const {
        return element(0);
      }
      std::string string_singleton() const {
        return element_as_string(0);
      }
    }; // TREX::transaction::BasicEnumerated
    
    
  } // TREX::transaction
} // TREX 

#endif // H_BasicEnumerated
