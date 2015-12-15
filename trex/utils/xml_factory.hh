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
#ifndef H_trex_utils_xml_factory
# define H_trex_utils_xml_factory

# include <utility>
# include <iostream>

# include "xml_utils.hh"
# include "generic_factory.hh"
# include "symbol.hh"

namespace trex {
  namespace utils {
    
    namespace internals {
      
      template<class Base, class Extra>
      struct xml_arg_helper {
      private:
#ifndef DOXYGEN
        typedef std::pair<Base *, Extra> computed_type;
        typedef std::pair<boost::property_tree::ptree::value_type *, Extra> computed_node;
#endif
      public:
        typedef computed_type argument_type;
        typedef computed_node node_proxy;
        
        static Base &xml(argument_type &arg) {
          return *(arg.first);
        }
        static argument_type build(Base &n, Extra &e) {
          return std::make_pair(&n, e);
        }
        
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
    
    
    template<class Product, class Output=Product *, class Arg=void>
    class xml_factory :boost::noncopyable {
      typedef boost::property_tree::ptree tree_t;
      
    public:
      typedef internals::xml_arg_helper<tree_t::value_type, Arg> arg_traits;
      
      template<class Iter>
      struct iter_traits :public internals::xml_arg_helper<Iter, Arg> {
#ifdef DOXYGEN
        using argument_type = typename internals::xml_arg_helper<Iter, Arg>::argument_type;
        typedef argument_type type;
#else
        typedef typename internals::xml_arg_helper<Iter, Arg>::argument_type type;
#endif
      }; // TREX::utils::XmlFactory<>::iter_traits<>
      
      typedef typename arg_traits::argument_type argument_type;
      
      typedef generic_factory<Product, symbol,
      argument_type, Output> factory_type;
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
      class declare :public xml_factory::factory_type::template declare<Ty>
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
      void get_ids(std::list<symbol> &ids) const {
        m_factory->get_ids(ids);
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
      xml_factory() {}
      ~xml_factory() {}
      
      singleton::use<factory_type> m_factory;
      
      friend class singleton::wrapper<xml_factory>;
    }; // TREX::utils::XmlFactory<>
    
    
# define In_H_trex_utils_xml_factory
#  include "bits/xml_factory.tcc"
# undef In_H_trex_utils_xml_factory
    
  } // TREX::utils
} // TREX

#endif // H_trex_utils_xml_factory
