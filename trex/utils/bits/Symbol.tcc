/* -*- C++ -*- */
/** @file "Symbol.tcc"
 * @brief BasicSymbol implementation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
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

template<class CharT, class Traits, class Alloc> 
std::ostream &
BasicSymbol<CharT, Traits, Alloc>::print_to(std::ostream &out) const {
  if( !empty() )
    out<<str();
  return out;
}

template<class CharT, class Traits, class Alloc> 
std::istream &BasicSymbol<CharT, Traits, Alloc>::read_from(std::istream &in) {
  str_type tmp;
  in>>tmp;
  if( in.good() )
    m_name = create(tmp);
  return in;
}



#endif 
