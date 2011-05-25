/* -*- C++ -*- */
/** @file "XmlFactory.tcc"
 * @brief XmlFactory implementation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */
#ifndef In_H_XmlFactory
# error "tcc files cannot be included outside of their corresponding header"
#else 

template<class Product, class Output, class Arg>
Output XmlFactory<Product, Output, Arg>::produce
(typename XmlFactory<Product, Output, Arg>::argument_type arg) {
  rapidxml::xml_node<> const &node = arg_traits::xml(arg);
  // The name of the tag identifies the producer
  Symbol id(node.name(), node.name_size());
  // Now I locate the producer for id and pass to him
  // the node and extra argument
  try {
    return m_factory->get(id)(arg);
  } catch(std::exception const &e) {
    // Capture any exception derived from std::exception
    // this includes TREX exceptions as rapidxml::parse_error
    throw XmlError(node, e.what());
  }  catch(...) {
    // capture unknown exceptions
    throw XmlError(node, "Unknown exception caught.");
  } 
}

template<class Product, class Output, class Arg>
bool XmlFactory<Product, Output, Arg>::iter_produce
(typename XmlFactory<Product, Output, Arg>::iter_type it, 
Output &ret) {
  ext_iterator &i = iter_traits::xml(it);

  while( i.valid() ) {
    Symbol tag(i->name(), i->name_size());
    if( m_factory->exists(tag) ) {
      argument_type arg = iter_traits::build_node(*i, it);
      try {
	ret = m_factory->get(tag)(arg);
      } catch(std::exception const &e) {
	// Capture any exception derived from std::exception
	// this includes TREX exceptions as rapidxml::parse_error
	throw XmlError(*i, e.what());
      }  catch(...) {
	// capture unknown exceptions
	throw XmlError(*i, "Unknown exception caught.");
      }
      ++i;
      return true;
    } else
      ++i;
  }
  return false;
}
 
#endif 
