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

      template<class Base, class Extra>
      struct xml_arg_helper {
      private:
	typedef std::pair<Base *, Extra> computed_type;
	typedef std::pair<boost::property_tree::ptree::value_type *, Extra> computed_node;

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
      }; // TREX::utils::internals::xml_arg_helper<>
      

    } // TREX::utils::details


    template<class Product, class Output=Product *, class Arg=void>
    class XmlFactory :boost::noncopyable {
      typedef boost::property_tree::ptree tree_t;

    public:
      typedef internals::xml_arg_helper<tree_t::value_type, Arg> arg_traits;
      
      template<class Iter>
      struct iter_traits :public internals::xml_arg_helper<Iter, Arg> {
	typedef typename internals::xml_arg_helper<Iter, Arg>::argument_type type;
      };

      typedef typename arg_traits::argument_type argument_type;
      
      typedef Factory<Product, Symbol, argument_type, Output> factory_type;
      typedef typename factory_type::returned_type returned_type;
      

      static tree_t::value_type &node(argument_type arg) {
	return arg_traits::xml(arg);
      }

      template<class Ty>
      class declare :public XmlFactory::factory_type::template declare<Ty> {
      public:
	declare(Symbol const &id) 
	  :factory_type::template declare<Ty>(id) {}
	virtual ~declare() {}
      }; // TREX::utils::XmlFactory<>::declare<>

      Output operator()(argument_type arg) {
	return produce(arg);
      }
      Output produce(argument_type arg);

      void getIds(std::list<Symbol> &ids) const {
	m_factory->getIds(ids);
      }

      template<class Iter>
      bool iter_produce(typename iter_traits<Iter>::type it, 
			Iter const &last, Output &ret);

    private:
      XmlFactory() {}
      ~XmlFactory() {}
      
      SingletonUse<factory_type> m_factory;

      friend class SingletonWrapper<XmlFactory>;
    }; // TREX::utils::XmlFactory<>


# define In_H_XmlFactory
#  include "bits/XmlFactory.tcc"
# undef In_H_XmlFactory

  } // TREX::utils
} // TREX

#endif // H_XmlFactory
