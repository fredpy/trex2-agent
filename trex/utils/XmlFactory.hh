/* -*- C++ -*- */
/** @file "XmlFactory.hh"
 *
 * @brief Factory based XML parsing utilities
 *
 * This header defines the XmlFactory class which helps user
 * defines a factory based on XML parsing
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
#ifndef H_XmlFactory
# define H_XmlFactory

# include <utility>
# include <iostream>

# include "XmlUtils.hh" 
# include "Factory.hh"
# include "Symbol.hh"

namespace TREX {
  namespace utils {

    namespace internals {

      /** @brief XmlFactory argument helper
       *
      * @tparam Base The argument bmain type. This type is often a 
       *   Boost.PropertyTree or a type including such tree
       * @tparam Extra Extra information type. The type emebedding no XML information
       *
       * This class is used by XmlDFactory as an helper to extract information from 
       * the factory production argument.  
       * 
       * @relates TREX::utils::XmlFactory
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup utils
       */
      template<class Base, class Extra>
      struct xml_arg_helper {
      private:
#ifndef DOXYGEN
	typedef std::pair<Base *, Extra> computed_type;
	typedef std::pair<boost::property_tree::ptree::value_type *, Extra> computed_node;
#endif 
      public:
        /** @brief Deduced argument type
         * The type used to compose @p Base and @p Extra. 
         * If @p Extra is @c void then this type is @p Base,
         * otherwise it would be a pair of the two types
         */
	typedef computed_type argument_type;
        /** @brief Deduced node type
         * The type used to compose a Boost.PropertyTree and @p Extra. 
         * This type is the one that the `XmlFactory` will pass to its producers   
         */
	typedef computed_node node_proxy;

        /** @brief Extract xml information
         *
         * @param[in] arg An argument structure
         *
         * This method allows to extract the @p Base component of @p arg. As @p Base 
         * is often related to a Boost.PropertyTree, we assume that this part is the XML 
         * part of the argument
         *
         * @return The @p Base comsponent of @p arg
         */
	static Base &xml(argument_type &arg) {
	  return *(arg.first);
	}
        /** @brief Build new element 
         *
         * @param[in] n A @p Base instance 
         * @param[in] e A @p Extra instance 
         *
         * Create a new argument
         *
         * @return the newly created argument that composes @p n and @p e 
         */
	static argument_type build(Base &n, Extra &e) {
	  return std::make_pair(&n, e);
	}

        /** @brief Build new node 
         *
         * @param[in] n A Boost.PropertyTree 
         * @param[in] b An  
         *
         * Create a new xml node
         *
         * @return the newly created node that composes @p n and the @p Extra part of @p b 
         */
	static node_proxy build_node(boost::property_tree::ptree::value_type &n,
				     argument_type &b) {
	  boost::property_tree::ptree::value_type *p = &n;
	  return std::make_pair(p, b.second);
	}
      }; // TREX::utils::internals::xml_arg_helper<>

#ifndef DOXYGEN
      template<class Base>
      struct xml_arg_helper<Base, void> {
	typedef Base                                    &argument_type;
	typedef boost::property_tree::ptree::value_type &node_proxy;

	static Base &xml(argument_type arg) {
	  return arg;
	}

	static argument_type build(Base &n) {
	  return n;
	}

	static node_proxy build_node(boost::property_tree::ptree::value_type &n,
				     argument_type b) {
	  return n;
	}
      }; // TREX::utils::internals::xml_arg_helper<,void>
#endif // DOXYGEN    

    } // TREX::utils::details


    /** @brief A Generic XML based factory
     *
     * @tparam Product The base type for the factory products
     * @tparam Output  The type returned by the factory
     * @tparam Arg     Optional extra type to be given with the Xml tree
     *
     * This template class implments a factory that can create new @p Product 
     * based on an XML description -- represented as a Boost.PropertyTree -- and 
     * an optional extra object of type @p Arg.
     *
     * It is a wrapper of the template class Factory that ease both the declaration 
     * and provides extra utilities that takes benefit of the XML structure.
     *
     * @sa Factory
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    template<class Product, class Output=Product *, class Arg=void>
    class XmlFactory :boost::noncopyable {
      typedef boost::property_tree::ptree tree_t;

    public:
      typedef internals::xml_arg_helper<tree_t::value_type, Arg> arg_traits;
      
      /** @brief Production iterator traits
       *
       * The traits used to identify the type of a production iterator for this 
       * factory based on a property_tree iterator
       *
       * @relates XmlFactory
       */
      template<class Iter>
      struct iter_traits :public internals::xml_arg_helper<Iter, Arg> {
#ifdef DOXYGEN
        // This is C++0x ... most compiler do not support it yet 
        //  for now we just put it for DOXYGEN asthis code is more parsable for it
        using argument_type = typename internals::xml_arg_helper<Iter, Arg>::argument_type;
        /** @brief iteration proxy type
         *
         * An alias to the type XmlFactory will use to iter_produce using 
         * @p Iter as a property iterator
         */
        typedef argument_type type; 
#else 
	typedef typename internals::xml_arg_helper<Iter, Arg>::argument_type type;
#endif
      }; // TREX::utils::XmlFactory<>::iter_traits<>

      typedef typename arg_traits::argument_type argument_type;
      
      /** @brief Subjacent factory
       *
       * The factory used underneath to implement this XmlFactory
       */
      typedef Factory<Product, symbol, argument_type, Output> factory_type;
      typedef typename factory_type::returned_type returned_type;
      

      static tree_t::value_type &node(argument_type arg) {
	return arg_traits::xml(arg);
      }

      /** @brief Simple producer declaration
       *
       * @tparam Ty the real product type
       *
       * This class allows to declare new producers for a XmlFactory and is based on 
       * Factory::declare implemetation.
       * The production is simply calling the constructor of @p Ty
       * accepting XmlFactory::argument_type as an argument.
       *
       * This producer wil be available in the Factory during its full lifetime
       * and will automatically unregister on destruction.
       *
       * @pre @p Ty has a constructor that accepts @p XmlFactory::argument_type has an argument
       * @pre @p Ty derives from @p Product
       * @pre @p Ty is a concrete class (no pure virtual method)
       *
       * @author Frederic Py <fpy@mbari.org>
       * @relates XmlFactory
       * @sa Factory::declare
       * @ingroup utils
       */
      template<class Ty>
#ifndef DOXYGEN
      class declare :public XmlFactory::factory_type::template declare<Ty> 
#else 
      class declare :public factory_type::declare<Ty> 
#endif 
      {
      public:
        /** @brief Constructor
         * @param[in] id A tag name
         *
         * Declare the new producer that will create a class Ty whenever the 
         * XmlFactory encounters an XML tag @p id
         */
	declare(symbol const &id)
	  :factory_type::template declare<Ty>(id) {}
        /** @brief Destructor 
         *
         * Undeclare this producer
         */
	virtual ~declare() {}
      }; // TREX::utils::XmlFactory<>::declare<>

      /** @brief Production operator
       *
       * @param[in] arg An XML argument
       *
       * Create a new instance based on @p arg
       * @{
       */
      Output operator()(argument_type arg) {
	return produce(arg);
      }
      Output produce(argument_type arg);
      /** @} */
       
      /** @brief Recognized XML tags
       *
       * @param[out] ids A list
       *
       * Store all the currently recognized XML tags for this factory in @p ids
       */
      void getIds(std::list<symbol> &ids) const {
	m_factory->getIds(ids);
      }

      /** @brief iterator based production
       *
       * @param[in,out] it   A production iterator
       * @param[in] last An end iterator
       * @param[out] ret product palcaholder 
       *
       * This method allow to create producats by iteratating through a single 
       * level of the XML structure.
       *
       * It advances @p it until a recognized tag is found or it reaches @p end.
       * If a recognized tag is found it the produces a new product and strores 
       * it in @p ret
       *
       * @throw XmlError captured an exception while trying to create a new product.
       *
       * @retval true if a production tag was found and @p ret has been updated
       * @retval false if @p it reached @p last without recognizing any tags
       */
      template<class Iter>
      bool iter_produce(typename iter_traits<Iter>::type it, 
			Iter const &last, Output &ret);

    private:
      XmlFactory() {}
      ~XmlFactory() {}
      
      singleton::use<factory_type> m_factory;

      friend class singleton::wrapper<XmlFactory>;
    }; // TREX::utils::XmlFactory<>


# define In_H_XmlFactory
#  include "bits/XmlFactory.tcc"
# undef In_H_XmlFactory

  } // TREX::utils
} // TREX

#endif // H_XmlFactory
