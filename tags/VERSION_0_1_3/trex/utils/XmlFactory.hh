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
#ifndef H_XmlFactory
# define H_XmlFactory

# include <utility>
# include <iostream>

# include "XmlUtils.hh" // avoid conflict with Europa 
# include "Factory.hh"
# include "Symbol.hh"

namespace TREX {
  namespace utils {

    namespace internals {
      
      /** @brief Argument type helper for XmlFactory
       * 
       * @tparam Base the base type for the node
       * @tparam Extra An extra argument type
       *
       * This class helps to identify and manipulate the argument type
       * of an XmlFactory class.
       *
       * XmlFactory class allows user to add an extra argument to be given
       * with the XML node during analysis. This comes handy when one need
       * extra information not directly accessible through the xml node.
       * This class is used by XmlFactory to identify the resulting argument
       * type used to produce a new object and access to the XML node.
       * In particular it take care of the special type where @p Extra is
       * @c void reflecting that no extra argument is expected apart the XML
       * node
       *
       * @note this class is an helper used internally by XmlFactory and
       * should not be used directly otherwise.
       *
       * @relates XmlFactory 
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup utils
       */
      template<class Base, class Extra>
      struct XmlArgHelper {
      private:
	typedef std::pair<Base *, Extra> computed_type;
	typedef std::pair<rapidxml::xml_node<> *, Extra> computed_node;

      public:
	/** @brief Argument type
	 *
	 * The type that the XmlFactory is expecting as argument to produce
	 * new object.
	 * @retval rapidxml::xml_node<> if @p Extra is @c void
	 * @retval std::pair<rapidxml::xml_node<>, Extra> otherwise
	 */
	typedef computed_type argument_type;
	
	/** @brief The type of a simple XmlFactory argument */
	typedef computed_node node_proxy;

	/** @brief XML node access
	 * 
	 * @param[in] arg A XmlFactory argument
	 * 
	 * This methods is an helper to access to the XML node
	 * encapsulated in @p arg
	 * 
	 * @return A reference to the XML node attached to @a arg
	 */
	static Base &xml(argument_type &arg) {
	  return *(arg.first);
	}

	/** @brief Construction helper
	 * 
	 * @param[in] n A @p Base instance
	 * @param[in] e An @p Extra instance
	 *
	 * Construct an argument_type using @p n and @p e.
	 *
	 * @return The structure embedding @p n and @p e
	 */
	static argument_type build(Base &n, Extra &e) {
	  return std::make_pair(&n, e);
	}

	/** @brief Construction helper
	 * 
	 * @param[in] n A XML node
	 * @param[in] b an argument_type from this traits
	 *
	 * Construct a node_proxy using @p n and the extra
	 * parameter embedded in @p b
	 *
	 * @return The constructed object
	 */
	static node_proxy build_node(rapidxml::xml_node<> &n,
				     argument_type &b) {
	  rapidxml::xml_node<> *p = &n;
	  return std::make_pair(p, b.second);
	}
	
      }; // TREX::utils::internals::XmlArgHelper<>

      /** @overloads struct XmlArgHelper<Base, Extra>
       *
       * This is the specialization for when no extra argument is given
       */
      template<class Base>
      struct XmlArgHelper<Base, void> {
	typedef Base &argument_type;

	static Base &xml(argument_type arg) {
	  return arg;
	}

	static rapidxml::xml_node<> &build_node(rapidxml::xml_node<> &n,
						argument_type b) {
	  return n;
	}

	static argument_type build(Base &n) {
	  return n;
	}

      }; // TREX::utils::internals::XmlArgHelper<,void> 
      
    } // TREX::utils::internals


    /** @brief Xml based factory
     * 
     * @tparam Product base type of the produced objects
     * @tparam Output  returned type by the production
     * @tparam Arg     type of the extra argument to give with the XML node
     *
     * This class uses the Factory template class to make a XML based factory.
     * This XML factory generate objects deriving from @p Product in
     * the pointer-like object @p Output and allows to give extra information
     * through the optional type @p Arg
     *
     * @author Frederic Py <fpy@mbari.org>
     *
     * @sa Factory
     * @sa internals::XmlArgHelper
     * @ingroup utils
     */
    template<class Product, class Output=Product *, class Arg=void>
    class XmlFactory :boost::noncopyable {
    public:
      /** @brief factory argument helper */
      typedef internals::XmlArgHelper<rapidxml::xml_node<>, Arg> arg_traits;
      /** @brief XML iterator argument helper */
      typedef internals::XmlArgHelper<ext_iterator, Arg> iter_traits;
      /** @brief argument type for the factory */
      typedef typename arg_traits::argument_type argument_type;
      /** @brief argument type for XML iterator */
      typedef typename iter_traits::argument_type iter_type;
      /** @brief subjacent factory type */
      typedef Factory<Product, Symbol, argument_type, Output> factory_type;
      /** @brief Type returned by the factory */
      typedef typename factory_type::returned_type returned_type;


      /** @brief XML node extraction
       *
       * @param[in] arg A factory argument
       *
       * This methods extract the XML content mbedded in @p arg.
       *
       * Depending on @p Arg this access can change and is given by arg_traits.
       * This mthods gives implementers an easy way to access it directly.
       *
       * @return The XML node embedded in @p arg
       *
       * @sa internals::XmlArgHelper
       */
      static rapidxml::xml_node<> &node(argument_type arg) {
	return arg_traits::xml(arg);
      }

      /** @brief Producer declaration
       * 
       * @tparam Ty concrete type of the product
       *
       * This class is a producer for the XmlFactory allowing the creation
       * of @p Ty instances.
       *
       * @pre @p Ty derives from @p Product
       * @pre @p Ty has a constructor that accepts XmlFactory::argument_type
       *      as argument.
       *
       * @sa class Factory::declare
       */
      template<class Ty>
      class declare :public XmlFactory::factory_type::template declare<Ty> {
      public:
	/** @brief Constructor
	 * 
	 * @param[in] id An identifier
	 *
	 * Creates a new instance and register it to the XmlFactory with
	 * the identifier @p id
	 *
	 * @pre @p id has not been already declared in the XmlFactory
	 *
	 * @throw MultipleFactoryDecl @p id is alreaduy in use in
	 *                            the XmlFactory
	 */
	declare(Symbol const &id) 
	  :factory_type::template declare<Ty>(id) {
	}
	/** @brief Destructor */
	virtual ~declare() {
	}
      }; // TREX::utils::XmlFactory<>::declare<>

      /** @brief production method
       * 
       * @param[in] arg An argument embedding the XML node and potentially
       *                extra information of type @p Arg
       *
       * This method is called to produce a new object from an xml node.
       * The type of the producer (or ID) is identified through the name
       * of the XML tag
       *
       * @throw XmlError an error occurred while trying to extract information
       *                 from the XML structure
       *                 
       * @note This method captures all exceptions and convert them to
       *       an XmlError. This was done for the following reasons :
       *       @li One single kind of exception is easier to handle
       *       @li as any exception is transformed into an XmlError, the
       *           programmer of the called constructor can produce
       *           whichever exception he wants while being sure that,
       *           when called by the Factory this exception will be
       *           transformed into an XmlError
       *       @li provided A cascading set of XmlError the output
       *           message will give a rough idea on the tag hierarchy
       *           that produced the error.
       * 
       * @return The resulting product
       * @{
       */
      Output operator()(argument_type arg) {
	return produce(arg);
      }
      Output produce(argument_type arg);
      /** @} */

      /** @brief Get available producers
       *
       * @param[out] ids The list of available producers
       *
       * This methods adds to @p ids all the identifier that are
       * currently available in this Factory
       */
      void getIds(std::list<Symbol> &ids) const {
	m_factory->getIds(ids);
      }

      /** @brief iterative creation
       * 
       * @param[in,out]  it   an iterator structure
       * @param[out] ret variable to store product
       *
       * While mebedded in a loop, this method allows to
       * iterate through an XML structure and call one of
       * the producers available in this factory any time
       * a valid XML tag is met.
       *
       * It advances @p it until the XML tage it is pointing to
       * corresponds to an existing identifier of the factory and
       * produces the corresponfing product which will be stored
       * into @p ret. After this operation @p it is pointing to
       * the next XML structure.
       *
       * @throw XmlError An error occured while trying to produce
       *                 the new output. The exception refers to
       *                 the XML element which was passed to the
       *                 production method
       *                 
       * @retval true  A new product is in @p ret and @p it has advanced
       * @retval false No production made. @p is not valid anymore
       *
       * @sa produce(argument_type)
       * @sa class ext_iterator
       */
      bool iter_produce(iter_type it, Output &ret);
    private:
      /** @brief Constructor */
      XmlFactory() {}
      /** @brief Destructor */
      ~XmlFactory() {}

      /** @brief Subjacent factory access
       */
      SingletonUse<factory_type> m_factory;

      friend class SingletonWrapper<XmlFactory>;
    }; // XmlFactory

# define In_H_XmlFactory
#  include "bits/XmlFactory.tcc"
# undef In_H_XmlFactory

  } // TREX::utils
} // TREX

#endif // H_XmlFactory
