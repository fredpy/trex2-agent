/* -*- C++ -*- */
/** @file IntervalDomain.tcc
 * @brief IntervalDomain class implemetation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup domains
 */
#ifndef In_H_IntervalDomain
# error "Cannot include template files outside of their corresponding header"
#else 

/*
 * class IntervalDomain<>::bound
 */ 

// statics :

template<typename Ty, class Cmp>
Cmp IntervalDomain<Ty, Cmp>::bound::s_cmp;    

// observers :
    
template<typename Ty, class Cmp>
std::ostream &
IntervalDomain<Ty, Cmp>::bound::print_to(std::ostream &out)  const {
  if( m_inf )
    out<<(m_pos?'+':'-')<<"inf";
  else {
    out.setf(std::ios::fixed);
    out.precision(2); // 2 significant digits
    out<<m_value;
  }
  return out;
}

template<typename Ty, class Cmp>
typename IntervalDomain<Ty, Cmp>::bound 
IntervalDomain<Ty, Cmp>::bound::plus
(typename IntervalDomain<Ty, Cmp>::bound const &other,
 typename IntervalDomain<Ty, Cmp>::bound const &onInf) const {
  if( m_inf || other.m_inf )
    return onInf;
  else 
    return bound(m_value+other.m_value);
}

template<typename Ty, class Cmp>
typename IntervalDomain<Ty, Cmp>::bound 
IntervalDomain<Ty, Cmp>::bound::minus
(typename IntervalDomain<Ty, Cmp>::bound const &other,
 typename IntervalDomain<Ty, Cmp>::bound const &onInf) const {
  if( other.m_inf || m_inf )
    return onInf;
  else 
    return bound(m_value-other.m_value);
}

// modifiers :

template<typename Ty, class Cmp>
std::istream &IntervalDomain<Ty, Cmp>::bound::read_from(std::istream &in) {
  // Attempt first to get [+-]inf
  bool is_pos;
  std::string buff;
  char c = in.peek();
  
  // check for optional sign character
  is_pos = c!='-';
  if( !is_pos || '+'==c ) {
    in.get(c);
    buff.insert(0, 1, c);
  }
  // look for "inf" case-insensitive
  static char const infstr[] = "INF";
  char const *i;
  i = infstr;
  while( *i && in.get(c) ) {
    buff.insert(0, 1, c);
    if( *i!=std::toupper(c) ) 
      break;
    ++i;
  }
  if( '\0'==*i ) {
    // success : set the bound to the corresponding set
    m_pos = is_pos;
    m_inf = true;
  } else {
    // failure : rollback and try to read Ty
    Ty val;
    for(size_t j=0; j<buff.length(); ++j )
      in.putback(buff[j]);
    in>>val;
    if( !in.bad() ) {
      // Alright it was a Ty value
      m_inf = false;
      m_value = val;
    }
  }
  return in;
}

/*
 * class IntervalDomain<>
 */ 

// statics :

template<typename Ty, class Cmp>
typename IntervalDomain<Ty, Cmp>::bound const 
IntervalDomain<Ty, Cmp>::plus_inf(true, true);

template<typename Ty, class Cmp>
typename IntervalDomain<Ty, Cmp>::bound const 
IntervalDomain<Ty, Cmp>::minus_inf(false, true); 

// observers :

template<typename Ty, class Cmp>
bool IntervalDomain<Ty, Cmp>::intersect(DomainBase const &other) const {
  if( getTypeName()!=other.getTypeName() ) 
    return false;
  else {
    IntervalDomain<Ty, Cmp> const &ref 
      = dynamic_cast<IntervalDomain<Ty, Cmp> const &>(other);
    return !(m_upper.min(ref.m_upper)<m_lower.max(ref.m_lower));
  }
}

template<typename Ty, class Cmp>
bool IntervalDomain<Ty, Cmp>::equals(DomainBase const &other) const {
  if( getTypeName()!=other.getTypeName() ) 
    return false;
  else {
    IntervalDomain<Ty, Cmp> const &ref 
      = dynamic_cast<IntervalDomain<Ty, Cmp> const &>(other);
    return m_lower==ref.m_lower && m_upper==ref.m_upper;
  }
}



// modifiers :
template<typename Ty, class Cmp>
void IntervalDomain<Ty, Cmp>::parseLower(std::string const &val) {
  bound tmp = TREX::utils::string_cast<bound>(val);
  if( m_upper<tmp || 
      ( m_upper==tmp && m_upper.isInfinity() ) ) 
    throw EmptyDomain(*this, "trying to set lower bound above upper bound");
  m_lower = tmp;
}

template<typename Ty, class Cmp>
void IntervalDomain<Ty, Cmp>::parseUpper(std::string const &val) {
  bound tmp = TREX::utils::string_cast<bound>(val);
  if( m_lower>tmp || 
      ( m_lower==tmp && m_lower.isInfinity() ) ) 
    throw EmptyDomain(*this, "trying to set upper bound below upper bound");
  m_upper = tmp;
}

template<typename Ty, class Cmp>
DomainBase &IntervalDomain<Ty, Cmp>::restrictWith
(IntervalDomain<Ty, Cmp>::bound const &lo,
 IntervalDomain<Ty, Cmp>::bound const &hi) {
  bound const &low = m_lower.max(lo);
  bound const &up = m_upper.min(hi);
  if( up<low )
    throw EmptyDomain(*this, "intersection is empty.");
  m_lower = low;
  m_upper = up;
  return *this;
}


template<typename Ty, class Cmp>
DomainBase &IntervalDomain<Ty, Cmp>::restrictWith(DomainBase const &other) {
  if( getTypeName()!=other.getTypeName() )
    throw EmptyDomain(*this, "Incompatible types");
  else {
    IntervalDomain<Ty, Cmp> const &ref 
      = dynamic_cast<IntervalDomain<Ty, Cmp> const &>(other);
    return restrictWith(ref.lowerBound(), ref.upperBound());
  }
  return *this;
}


#endif 
