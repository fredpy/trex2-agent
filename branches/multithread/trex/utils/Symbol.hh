/* -*- C++ -*- */
/** @file "Symbol.hh"
 *
 * Symbol class deintion and utilities
 *
 * This header defines the Symbol class. Symbol is a class based on
 * standard C++ string but managed via  a unique instance reference
 * counting pointer ensuring better performances and memory management
 * for constant strings frequenlty used.
 *
 * @note In this latest implementation this class is just some wrapper
 * of @c boost::flyweight class which ewas kept just toavoid heavy
 * modifications of the client classes.
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
#ifndef H_Symbol 
# define H_Symbol

# include <string>

# include <boost/flyweight.hpp>
# include <boost/flyweight/no_tracking.hpp>
# include <boost/flyweight/holder_tag.hpp>

# include "Hashable.hh"
# include <boost/lexical_cast.hpp>
# include "singleton.hh"

namespace TREX {
  namespace utils {
    
    template<typename C>
    class trex_holder_class {
    public:
      static C& get() {
        static singleton_use<C> hold;
        return *hold;
      }
    };
    
    struct trex_holder_specifier :boost::flyweights::holder_marker {
      template<typename C>
      struct apply {
        typedef trex_holder_class<C> type;
      };
    };
    
  }
}

namespace boost {
  namespace flyweights {
    
    template<>
    struct is_holder<TREX::utils::trex_holder_specifier> :boost::mpl::true_ {};
    
  }
}

namespace TREX {
  namespace utils {


    /** @brief generic symbol class
     *
     * @param CharT character type
     * @param Traits character manipulation traits for @e CharT
     * @param Alloc  A memory allocator
     *
     * This class offer a generic symbol class definition.
     * A symbol is a a string access point ensuring that there's
     * no more than one string value stored in memory. It relies
     * on a unique reference memory managed by a reference counter
     * to ensure both equality efficiency and better memory management.
     *
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    template< class CharT, class Traits=std::char_traits<CharT>,
	      class Alloc=std::allocator<CharT> >
    class BasicSymbol :public Hashable {
    public:
      /** @brief equivalent string type */
      typedef std::basic_string<CharT, Traits, Alloc> str_type;
      
    private:
      /** @brief the symbol value
       *
       * @note I have purposedly set the policy to @c no_tracking.
       * This implies that all symbol created will remain in
       * memory until the end of the program. This make sense as
       * these symbols will be used all along the agent lifetime
       * but if you think it implies an increasing memory usage you
       * can change it back by removing this.
       * @note @c intermodule_holder is used to support dynmic
       * libraries loading (and/or plug-ins)
       */
      typedef typename 
        boost::flyweight< str_type,
			  boost::flyweights::no_tracking,
                         boost::flyweights::holder<trex_holder_specifier> > ref_type;

    public:
      /** @brief Constructor
       *
       * @param str a string
       *
       * Create a new symbol with the same value as @e str
       */
      BasicSymbol(str_type const &str = str_type()) 
	:m_name(create(str)) {}
      /** @brief Constructor
       *
       * @param str a string
       * @param len A length
       *
       * Create a new symbol with the value of @e str up to its @e len
       * character.
       *
       * @sa BasicSymbol &reset(CharT const *str, size_t len)
       */
      BasicSymbol(CharT const *str, size_t len =str_type::npos)
	:m_name(create(str, len)) {}
      /** @brief Copy constructor
       *
       * @param other another instance
       *
       * Create a new symbol with the same value as @e other
       */
      BasicSymbol(BasicSymbol const &other)
	:m_name(other.m_name) {}
      /** @brief Destructor */
      ~BasicSymbol() {
      }

      /** @brief Asignment operator
       *
       * @param other another instance
       *
       * Copy the value of @e other into current instance
       *
       * @return current instance after operation
       */
      BasicSymbol &operator= (BasicSymbol const &other) {
	m_name = other.m_name;
	return *this;
      }

      /** @brief Eeset value
       *
       * @param str a string
       * @param len A length
       *
       * Change current instance value with the value of @e str up
       * to its @e len character.
       *
       * @sa BasicSymbol(CharT const *str, size_t len)
       */
      BasicSymbol &reset(CharT const *str=NULL, size_t len=str_type::npos) {
	m_name = create(str, len);
	return *this;
      }
      
      /** @brief Check for emptyness
       *
       * @retval true if length()==0
       * @retval false otherwise
       *
       * @sa size_t length() const
       */
      bool empty() const {
	return m_name.get().empty();
      }
      /** @brief Symbol length
       *
       * @return the length of the string representing this instance
       */
      size_t length() const {
	return size_t(empty()?0:m_name.get().size());
      }
      
      /** @brief Equality test
       *
       * @param other Another instance
       *
       * Test if current instance is equal to @e other. This method
       * relies on uniquer instance memory to ensure that this test
       * is done in constant time independently of the string length
       *
       * @retval true if current instance is equal to @e other
       * @retval false otherwise
       *
       * @sa bool operator!=(BasicSymbol const &) const
       */
      bool operator==(BasicSymbol const &other) const {
	return m_name==other.m_name;
      }
      /** @brief Difference test
       *
       * @param other Another instance
       *
       * Test if current instance is different from @e other. This method
       * relies on uniquer instance memory to ensure that this test
       * is done in constant time independently of the string length
       *
       * @retval true if current instance is not equal to @e other
       * @retval false otherwise
       *
       * @sa bool operator==(BasicSymbol const &) const
       */
      bool operator!=(BasicSymbol const &other) const {
	return !operator==(other);
      }
      /** @brief Less than test
       *
       * @param other instance
       *
       * test if current instance is less than @e other. In the
       * case these two are equals, the operation is accelerated
       * by the unique instance memory
       *
       * @retval true if current instance is before @e other
       * @retval false  otherwise
       */
      bool operator< (BasicSymbol const &other) const;
      /** @brief Greater than test
       *
       * @param other instance
       *
       * test if current instance is greater than @e other. In the
       * case these two are equals, the operation is accelerated
       * by the unique instance memory
       *
       * @retval true if current instance is after @e other
       * @retval false  otherwise
       */
      bool operator> (BasicSymbol const &other) const {
	return other.operator< (*this);
      }
      /** @brief Less or equal to test
       *
       * @param other instance
       *
       * test if current instance is less or equal @e other. In the
       * case these two are equals, the operation is accelerated
       * by the unique instance memory
       *
       * @retval false if current instance is after @e other
       * @retval true  otherwise
       */
      bool operator<=(BasicSymbol const &other) const {
	return !operator> (other);
      }
      /** @brief Greater or equal to test
       *
       * @param other instance
       *
       * test if current instance is greater or equal @e other. In the
       * case these two are equals, the operation is accelerated
       * by the unique instance memory
       *
       * @retval false if current instance is before @e other
       * @retval true  otherwise
       */
      bool operator>=(BasicSymbol const &other) const {
	return !operator< (other);
      }

      /** @brief String value
       *
       * @return the equivalent string value to this instance
       */
      str_type const &str() const {
	return m_name.get();
      }

    private:
      /** @brief symbol value */
      ref_type m_name;
      /** @brief Symbol creation
       *
       * @param str A string
       *
       * Create a new entry into the unique memory corresponding to
       * @e str
       *
       * @return The reference to the cell corresponding to @e str
       */
      static ref_type create(str_type const &str);
      /** @brief Symbol creation
       *
       * @param str A string
       * @param len a length
       *
       * Create a new entry into the unique memory corresponding to
       * @e str up to the @e len character.
       *
       * @return The reference to the cell corresponding 
       */
      static ref_type create(CharT const *str, size_t len);

      size_t hash() const;
      
      friend std::istream &operator>>(std::istream &in, BasicSymbol &s) {
        BasicSymbol::str_type tmp;
        
        if( in>>tmp )
          s.m_name = BasicSymbol::create(tmp);
        return in;
      }

    }; // TREX::utils::BasicSymbol<>

    template<class CharT, class Traits, class Alloc>
    std::ostream &operator<<(std::ostream &out, BasicSymbol<CharT, Traits, Alloc> const &s) {
      if( !s.empty() )
        out<<s.str();
      return out;
    }


    
# define In_H_Symbol
#  include "bits/Symbol.tcc"
# undef In_H_Symbol

    /** @brief char based BasicSymbol
     *
     * This typedef is the default type one would use to
     * manipulate symbols. It is the counterpart of the standard
     * C++ string class.
     *
     * @ingroup utils
     */
    typedef BasicSymbol<char> Symbol;
    
  } // TREX::utils
} // TREX 

#endif // H_Symbol
