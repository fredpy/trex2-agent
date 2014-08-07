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
#ifndef H_Factory
# define H_Factory

# include <map>
# include <iostream>
# include <list>

# include <boost/call_traits.hpp>
# include <boost/static_assert.hpp>
# include <boost/type_traits.hpp>
# include <boost/utility.hpp>

# include "SingletonUse.hh"

namespace TREX {
  namespace utils {
		
    /** @brief generic Factory exception
     *
     * This is the base class for exception thrown by the Factory class.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @relates class TREX::utils::Factory
     * @ingroup utils
     */
    class FactoryException :public Exception {
    public:
      /** @brief Destructor */
      virtual ~FactoryException() throw() =0;
    protected:
      /** @brief Constructor
       *
       * @param[in] msg A message
       *
       * Create a new instance with the associated message @p msg
       */
      FactoryException(std::string const &msg) throw() 
	:Exception(msg) {}
    }; // class TREX::utils::FactoryException
		
    inline FactoryException::~FactoryException() throw() {}
		
    /** @brief Unknown producer ID
     *
     * This exception is thrown by Factory when trying to access
     * to an undeclared Factory producer.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @relates class TREX::utils::Factory
     * @ingroup utils
     */
    class UnknownFactoryType :public FactoryException {
    public:
      /** @brief Constructor
       *
       * @param msg A message
       *
       * Create a new instance with the associated message @a msg
       */
      UnknownFactoryType(std::string const &msg) throw() 
	:FactoryException(msg) {}
      /** @brief Destructor */
      ~UnknownFactoryType() throw() {}
			
    }; // class TREX::utils::UnknownFactoryType
		
    /** @brief Multiple producers declaration
     *
     * This exception is thrown by Factory when trying to declare
     * multiple producers with the same associated ID.
     *
     * @note As most of the declaration are done using static global
     * variables. This exception is generally thrown outside of the
     * @c main function scope. I do not know how it would impact the
     * execution of the program in different platforms.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @relates class TREX::utils::Factory
     * @ingroup utils
     */
    class MultipleFactoryDecl :public FactoryException {
    public:
      /** @brief Constructor
       *
       * @param msg A message
       *
       * Create a new instance with the associated message @e msg
       */
      MultipleFactoryDecl(std::string const &msg) throw() 
	:FactoryException(msg) {}
      /** @brief Destructor */
      ~MultipleFactoryDecl() throw() {}
    }; // class TREX::utils::MultipleFactoryDecl
		
		
    /** @brief Generic global factory
     *
     * @tparam AbstractProduct Base type of the products
     * @tparam Id              Identifier type for the producers
     * @tparam ConsArg         Type of the argument passed for the
     *                         production of the new product.
     * @tparam ProductRef      The type returned by producer after creation
     * @tparam IdComp          Ordering functor between @e Id
     *
     * This class implements the factory design pattern. More
     * specifically class is used as a singleton class that
     * provides a global factory that any can use to produce
     * objects derived from @a AbstractProduct and having a
     * constructor with @a ConsArg as argument. The access
     * to producers is given by a key of type @p Id.
     *
     * The general idea of this class was to allow automatic
     * registering and un-registering of the producers during
     * their construction and destruction. Producers can be simply
     * defined using the Factory::declare template class. For
     * example in a Factory that produces abstract instances of @c Foo
     * which is the mother class of @c Bar that can be constructed by
     * passing an @c int as argument you can uses the following
     * code to declare a producer for @c Bar associating it to the
     * string "bar
     * @code
     * # include "Factory.hh"
     *
     * TREX::utils::Factory<Foo, std::string, int>::declare<Bar> 
     *      bar_decl("bar");
     * @endcode
     * As long as @c bar_decl is existing the factory will provide
     * an access to this producer. To be able to use it one can do :
     * @code
     * SingletonUse< Factory<Foo,std::string, int> > bar_fact;
     * Foo *var = bar_fact->get("bar")(5);
     * @endcode
     *
     * @sa TREX::utils::SingletonUse
     * 
     * @pre @p ProductRef is expected to be a pointer like type for
     * @p AbstractProduct
     * @pre @p IdComp should be a complete order functor other @p Id
     *
     * @note In order to ease the automatioc registration of producers
     *       the factory class is implemented as a Singleton. This allows
     *       the producers to know directly how to find their associated
     *       Factory. On the other hand you can't have easily multiple
     *       factories of the same type.
     *    
     * @note As for now Factory class relies on std::map to
     *       store the producers. As a result we need to have a complete
     *       order for @p Id. It may be better to switch in the future
     *       on C++ TR1 collections that would only require the @p Id type
     *       to provide hash and equality functor allowing incomplete order.
     * 
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    template< class AbstractProduct, class Id, class ConsArg,
	      class ProductRef = AbstractProduct *,
	      class IdComp = std::less<Id> >
    class Factory :boost::noncopyable {
    public:
      /** @brief Type for producers ID */
      typedef Id              producer_id;
      /** @brief Parameter type used for producers ID */
      typedef typename boost::call_traits<producer_id>::param_type id_param;
      /** @brief product base type */
      typedef AbstractProduct product_type;
      /** @brief returned type after production */
      typedef ProductRef      returned_type;
			
      /** @brief Get available producers
       *
       * @param[out] ids The list of available producers
       *
       * This methods adds to @p ids all the identifier that are
       * currently available in this Factory
       */
      void getIds(std::list<Id> &ids) const;
      
      /** @brief Abstract producer type
       *
       * This class provides an abstract interface to
       * manipulate a factory producer. This class behaves like
       * a functor that can be accessed through the Factory
       * access methods.
       *
       * @sa Factory::get(typename boost::call_traits<Id>::param_type) const
       *
       * @author Frederic Py <fpy@mbari.org>
       */
      class producer: boost::noncopyable {
      public:
	/** @brief Production argument type
	 *
	 * This type is used to pass the production argument
	 * to this functor to produce a new product
	 */
	typedef typename boost::call_traits<ConsArg>::param_type argument_type;
	/** @brief Production result
	 *
	 * This is the type of given as a result after production.
	 */
	typedef ProductRef result_type;
				
	/** @brief production operator
	 * @param[in] arg production argument
	 *
	 * Creates a new @p AbstractProduct using the
	 * argument @p arg.
	 *
	 * It calls the virtual method @c produce which
	 * implements the craetion of the new object.
	 *
	 * @return A @p ProductRef to the newly created product
	 * 
	 * @sa produce(argument_type) const
	 */
	result_type operator()(argument_type arg) const {
	  return produce(arg);
	}
				
	void notify() {
	  declareMe();
	}

	/** @brief get producer ID
	 *
	 * @return The ID associated to this producer
	 */
	Id const &getId() const {
	  return m_type;
	}
				
      protected:
	/** @brief production method
	 * 
	 * @param[in] arg production argument
	 *
	 * This method create a new product with the argument @p arg
	 *
	 * @return A @p ProductRef to the newly created product
	 * 
	 * @sa operator()(argument_type) const
	 */
	virtual result_type produce(argument_type arg) const =0;
				
	/** @brief Constructor
	 * @param[in] id An identifier
	 *
	 * Create a new instance identified by @p id
	 *
	 * @sa declareMe() const
	 */
	producer(id_param id)
	  :m_type(id) {}
				
	/** @brief Registration to the Factory
	 *
	 * This method registers current instance to the Factory
	 *
	 * @pre The identifier of this producer is not currently
	 * declared in the Factory
	 * @throw MultipleFactoryDecl Another producer has been
	 * declared in the Factory with the ID of this producer
	 *
	 * @sa producer(id_param) 
	 * @sa Factory::add(Factory::producer const *)
	 */
	void declareMe() const {
	  // std::cerr<<m_factory.operator->()<<".add("<<m_type<<")"<<std::endl;
	  m_factory->add(this);
	}
				
	/** @brief Destructor
	 *
	 * Unregister this instance from the Factory
	 *
	 * @sa Factory::remove(producer *)
	 */
	virtual ~producer() {
	  // std::cerr<<m_factory.operator->()<<".remove("<<m_type<<")"<<std::endl;
	  m_factory->remove(this);
	}
				
      private:
				
	Id const              m_type;    //!< Producer Identifer
	SingletonUse<Factory> m_factory; //!< Acees point to Factory singleton
				
	/** @brief Prohibited Default constructor
	 *
	 * @warning This constructor has no code
	 * associated and is set to private in order to avoid its
	 * call. The only construction mecanism allowed is by using
	 * the consructor with id_param s argument.
	 *
	 * @sa producer(id_param)
	 */
	producer();
      }; // class TREX::utils::Factory<>::producer
			
      /** @brief Simple producer declaration
       *
       * @tparam Product the real product type
       *
       *
       * This class is the simplest implementation for a Factory producer.
       * The production is simply calling the constructor of @p Product
       * accepting @p ConsArg as an argument.
       *
       * It is a simple illustration on how a concrete producer can register
       * and be manipulated by a Factory.
       *
       * This producer wil be available in the Factory during its full lifetime
       * and will automatically unregister on destruction.
       *
       * @pre @p Product has a constructor that accepts @p ConsArg has an argument
       * @pre @p Product derives from @p AbstractProduct
       * @pre @p Product is a concrete class (no pute virtual method)
       *
       * @author Frederic Py <fpy@mbari.org>
       * @relates Factory
       * @ingroup utils
       */
      template<class Product>
      class declare :public producer {
      public:
	/** @brief Constructor
	 * 
	 * @param[in] id An identifier
	 *
	 * Create a new instance and declare it to the Factory with the
	 * identifier @p id
	 *
	 * @pre The identifier @p id is not currently declared in the Factory
	 * 
	 * @throw MultipleFactoryDecl Another producer has been already
	 * declared in the Factory with the identifier @p id
	 *
	 * @sa Factory::producer::declareMe() const
	 */
	declare(id_param id) 
	  :producer(id) {
	  producer::notify();
	}


	/** @brief Destructor
	 *
	 * Unregister this instance from the Factory.
	 */	 
	virtual ~declare() {}
				
      private:
	typename producer::result_type produce(typename producer::argument_type arg) 
	  const {
	  typename producer::result_type ret(new Product(arg));
	  return ret;
	}
				
	/** @brief Compilation time concept checking
	 *
	 * Checks at compilation that @p Product is derived from
	 * @p AbstractProduct
	 */
	BOOST_STATIC_ASSERT((boost::is_convertible<Product *,
						   AbstractProduct *>::value));
				
      }; // class TREX::utils::Factory<>::declare<>
			
      /** @brief Producer access method
       *
       * @param[in] id A producer ID
       *
       * Gives access to the producer associated to @p id.
       *
       * @pre A producer has been registered with the identifier @p id
       * 
       * @return The producer associated to @p id
       * 
       * @throw UnknownFactoryType No producer associated to identifier @p id
       *
       * @sa exists(id_param id) const
       *
       * @{
       */
      producer const &get(id_param id) const;
      producer const &operator[](id_param id) const {
	return get(id);
      }
      /** @} */
			
      /** @brief Check for producer existence
       *
       * @param[in] id An identifier
       *
       * @retval true if there's a producer associated to @p id
       * @retval false otherwise
       */
      bool exists(id_param id) const; 
			
    private:
      /** @brief Constructor */
      Factory() {}
      
      /** @brief Destructor */
      ~Factory() {}
			
      typedef std::map<Id, producer const *, IdComp> catalog_type; //!< Producers catalog type
      
      /** @brief New producer registration
       *
       * @param[in] prod A producer
       *
       * This method register @p prod with its associated ID
       *
       * @pre The ID of @p prod should not be already in use
       * 
       * @throw MultipleFactoryDecl The ID of @p prod is already in
       * use.
       * 
       * @post @p prod is registered and any request for its ID will
       * provide an access to this producer.
       */
      void add(producer const *prod);
      
      /** @brief producer un-registration
       *
       * @param[in] prod A producer
       *
       * This method un-register @p prod with its associated ID
       * 
       * @post if @p prod was correctly registered it is now
       * unregistered and its associated ID is no longer available.
       */
      void remove(producer const *prod);
      
      /** @brief Producers catalog
       *
       * This attributes maintins the set of all the producers for this
       * Factory and associuate then to their corresponding @p Id
       */
      catalog_type m_producers;
			
      friend class producer;
      friend class SingletonWrapper<Factory>;
    }; // class TREX::utils::Factory<>
		
# define In_H_Factory
#  include "bits/Factory.tcc"
# undef In_H_Factory
		
  } // TREX::utils 
} // TREX

#endif // H_Factory
