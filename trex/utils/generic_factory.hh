/** @file "Factory.hh"
 * @brief Factory class definition
 * 
 * This header defines the Factory template class
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
#ifndef H_trex_utils_generic_factory
# define H_trex_utils_generic_factory

# include <map>
# include <iostream>
# include <list>

# include <boost/call_traits.hpp>
# include <boost/static_assert.hpp>
# include <boost/type_traits.hpp>
# include <boost/utility.hpp>

# include "singleton.hh"
# include "platform/cpp11_deleted.hh"
# include "Exception.hh"

namespace TREX {
  namespace utils {
		
    class FactoryException :public Exception {
    public:
      virtual ~FactoryException() throw() =0;
    protected:
      FactoryException(std::string const &msg) throw()
	:Exception(msg) {}
    }; // class TREX::utils::FactoryException
		
    inline FactoryException::~FactoryException() throw() {}
		
    class UnknownFactoryType :public FactoryException {
    public:
      UnknownFactoryType(std::string const &msg) throw()
	:FactoryException(msg) {}
      ~UnknownFactoryType() throw() {}
			
    }; // class TREX::utils::UnknownFactoryType

    class MultipleFactoryDecl :public FactoryException {
    public:
      MultipleFactoryDecl(std::string const &msg) throw()
	:FactoryException(msg) {}
      ~MultipleFactoryDecl() throw() {}
    }; // class TREX::utils::MultipleFactoryDecl
		
		
    template< class AbstractProduct, class Id, class ConsArg,
	      class ProductRef = AbstractProduct *,
	      class IdComp = std::less<Id> >
    class generic_factory :boost::noncopyable {
    public:
      typedef Id              producer_id;
      typedef typename boost::call_traits<producer_id>::param_type id_param;
      typedef AbstractProduct product_type;
      typedef ProductRef      returned_type;
			
      void get_ids(std::list<Id> &ids) const;
      
      class producer: boost::noncopyable {
      public:
	typedef typename boost::call_traits<ConsArg>::param_type argument_type;
	typedef ProductRef result_type;

	result_type operator()(argument_type arg) const {
	  return produce(arg);
	}
				
	void notify() {
	  declare_me();
	}

	Id const &get_id() const {
	  return m_type;
	}
				
      protected:
	virtual result_type produce(argument_type arg) const =0;
				
	producer(id_param id)
	  :m_type(id) {}
				
	void declare_me() const {
	  m_factory->add(this);
	}
				
	virtual ~producer() {
	  m_factory->remove(this);
	}
				
      private:
				
	Id const              m_type;
        singleton::use<generic_factory> m_factory;
        
	producer() DELETED;
      }; // class TREX::utils::generic_factory<>::producer
			
      template<class Product>
      class declare :public producer {
      public:
	declare(id_param id)
	  :producer(id) {
	  producer::notify();
	}

	virtual ~declare() {}
				
      private:
	typename producer::result_type produce(typename producer::argument_type arg) 
	  const {
	  typename producer::result_type ret(new Product(arg));
	  return ret;
	}
				
	BOOST_STATIC_ASSERT((boost::is_convertible<Product *,
						   AbstractProduct *>::value));
				
      }; // class TREX::utils::Factory<>::declare<>
			
      producer const &get(id_param id) const;
      producer const &operator[](id_param id) const {
	return get(id);
      }
			
      bool exists(id_param id) const;
			
    private:
      generic_factory() {}
      
      ~generic_factory() {}
			
      typedef std::map<Id, producer const *, IdComp> catalog_type;

      void add(producer const *prod);
      void remove(producer const *prod);
      
      catalog_type m_producers;
			
      friend class producer;
      friend class singleton::wrapper<generic_factory>;
    }; // class TREX::utils::generic_factory<>
		
# define In_H_trex_utils_generic_factory
#  include "bits/generic_factory.tcc"
# undef In_H_trex_utils_generic_factory
		
  } // TREX::utils 
} // TREX

#endif // H_Factory
