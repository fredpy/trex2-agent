/* -*- C++ -*- */
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
#ifndef In_H_trex_utils_generic_factory
# error "tcc files cannot be included outside of their corresponding header" 
#else									

template<class P, class I, class C, class R, class O>
typename generic_factory<P, I, C, R, O>::producer const &
generic_factory<P, I, C, R, O>::get
(typename generic_factory<P, I, C, R, O>::id_param id) const {
    typename generic_factory<P, I, C, R, O>::catalog_type::const_iterator
    i = m_producers.find(id);
    if( m_producers.end()==i ) {
      ERROR_CODE ec = factory_error::make_error(factory_error::unknown_id);
      throw SYSTEM_ERROR(ec);
    }
    return *(i->second);
}

template<class P, class I, class C, class R, class O>
bool generic_factory<P, I, C, R, O>::exists
(typename generic_factory<P, I, C, R, O>::id_param id) const {
    return m_producers.end()!=m_producers.find(id);
}

template<class P, class I, class C, class R, class O>
void generic_factory<P, I, C, R, O>::add
(typename generic_factory<P, I, C, R, O>::producer const *prod) {
    typename generic_factory<P, I, C, R, O>::catalog_type::value_type
    to_ins(prod->get_id(), prod);
    if( !m_producers.insert(to_ins).second ) {
# if 0 // I have a weird bug some time with that ... to be checked
        throw MultipleFactoryDecl("ID multiply used.");
# endif 
    }
}

template<class P, class I, class C, class R, class O>
void generic_factory<P, I, C, R, O>::remove
(typename generic_factory<P, I, C, R, O>::producer const *prod) {
    typename generic_factory<P, I, C, R, O>::catalog_type::iterator
    i = m_producers.find(prod->get_id());
    if( m_producers.end()!=i && i->second==prod ) 
        m_producers.erase(i);
}

template<class P, class I, class C, class R, class O>
void generic_factory<P, I, C, R, O>::get_ids(std::list<I> &ids) const {
    typename generic_factory<P, I, C, R, O>::catalog_type::const_iterator
    i = m_producers.begin();
    for( ; m_producers.end()!=i; ++i)
        ids.push_back(i->first);
}


#endif // In_H_Factory
