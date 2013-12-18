/* -*- C++ -*- */
/** @file "Symbol.tcc"
 * @brief BasicSymbol implementation
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
#ifndef In_H_Symbol
# error "tcc files cannot be included outside of their corresponding header"
#else

/*
 * class TREX::utils::BasicSymbol<>
 */ 
// statics
template<class CharT, class Traits, class Alloc> 
typename BasicSymbol<CharT, Traits, Alloc>::ref_type
BasicSymbol<CharT, Traits, Alloc>::create
(typename BasicSymbol<CharT, Traits, Alloc>::str_type const &str) {
  if( str.empty() )
    return ref_type();
  else 
    return ref_type(str);
}

template<class CharT, class Traits, class Alloc> 
typename BasicSymbol<CharT, Traits, Alloc>::ref_type
BasicSymbol<CharT, Traits, Alloc>::create(CharT const *str, size_t len) {
  if( NULL==str || 0==len )
    return ref_type();
  else if( str_type::npos==len )
    return create(str_type(str));
  else
    return create(str_type(str, len));
}

// observers

template<class CharT, class Traits, class Alloc> 
bool BasicSymbol<CharT, Traits, Alloc>::operator< 
(BasicSymbol<CharT, Traits, Alloc> const &other) const {
  return !other.empty() &&
    ( empty() ||( m_name!=other.m_name &&
		  m_name.get()<other.m_name.get() ) );
}

template<class CharT, class Traits, class Alloc> 
size_t BasicSymbol<CharT, Traits, Alloc>::hash() const {
  boost::hash<str_type> href;
  return href(m_name.get());
}


#endif 
