/* -*- C++ -*- */
/** @file "XmlFactory.tcc"
 * @brief XmlFactory implementation
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
#ifndef In_H_XmlFactory
# error "tcc files cannot be included outside of their corresponding header"
#else 

# ifndef DOXYGEN

template<class Product, class Output, class Arg>
Output xml_factory<Product, Output, Arg>::produce
(typename xml_factory<Product, Output, Arg>::argument_type arg) {
  tree_t::value_type &node = arg_traits::xml(arg);
  symbol id(node.first);

  return m_factory->get(id)(arg);
}

template<class Product, class Output, class Arg>
template<class Iter>
bool xml_factory<Product, Output, Arg>::iter_produce
(typename xml_factory<Product, Output, Arg>::template iter_traits<Iter>::type it,
 Iter const &last,
 Output &ret) {
  Iter &i = iter_traits<Iter>::xml(it);
  
  while( last!=i ) {
    symbol tag(i->first);
    if( m_factory->exists(tag) ) {
      argument_type arg = iter_traits<Iter>::build_node(*i, it);
      
      ret = m_factory->get(tag)(arg);
      ++i;
      return true;
    } else 
      ++i;
  }
  // No correct tag found
  return false;
}

# endif // DOXYGEN
 
#endif 
