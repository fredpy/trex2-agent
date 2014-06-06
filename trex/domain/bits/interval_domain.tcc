/* -*- C++ -*- */
/** @file IntervalDomain.tcc
 * @brief IntervalDomain class implemetation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup domains
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
#ifndef In_H_IntervalDomain
# error "Cannot include template files outside of their corresponding header"
#else 

/*
 * class interval_domain<>::bound
 */ 

// statics :

template<typename Ty, bool Prot, class Cmp>
Cmp interval_domain<Ty, Prot, Cmp>::bound::s_cmp;

// observers :
    
template<typename Ty, bool Prot, class Cmp>
std::ostream &
interval_domain<Ty, Prot, Cmp>::bound::print_to(std::ostream &out)  const {
  if( m_inf )
    out<<(m_pos?'+':'-')<<"inf";
  else 
    out<<m_value;
  return out;
}

template<typename Ty, bool Prot, class Cmp>
typename interval_domain<Ty, Prot, Cmp>::bound
interval_domain<Ty, Prot, Cmp>::bound::plus
(typename interval_domain<Ty, Prot, Cmp>::bound const &other,
 typename interval_domain<Ty, Prot, Cmp>::bound const &onInf) const {
  if( m_inf || other.m_inf )
    return onInf;
  else 
    return bound(m_value+other.m_value);
}

template<typename Ty, bool Prot, class Cmp>
typename interval_domain<Ty, Prot, Cmp>::bound
interval_domain<Ty, Prot, Cmp>::bound::minus
(typename interval_domain<Ty, Prot, Cmp>::bound const &other,
 typename interval_domain<Ty, Prot, Cmp>::bound const &onInf) const {
  if( other.m_inf || m_inf )
    return onInf;
  else 
    return bound(m_value-other.m_value);
}

// modifiers :

template<typename Ty, bool Prot, class Cmp>
std::istream &interval_domain<Ty, Prot, Cmp>::bound::read_from(std::istream &in) {
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
 * class interval_domain<>
 */ 

// statics :

template<typename Ty, bool Prot, class Cmp>
typename interval_domain<Ty, Prot, Cmp>::bound const
interval_domain<Ty, Prot, Cmp>::plus_inf(true, true);

template<typename Ty, bool Prot, class Cmp>
typename interval_domain<Ty, Prot, Cmp>::bound const
interval_domain<Ty, Prot, Cmp>::minus_inf(false, true);

// observers :

template<typename Ty, bool Prot, class Cmp>
bool interval_domain<Ty, Prot, Cmp>::intersect(abstract_domain const &other) const {
  if( type_name()!=other.type_name() )
    return false;
  else {
    interval_domain<Ty, Prot, Cmp> const &ref
      = dynamic_cast<interval_domain<Ty, Prot, Cmp> const &>(other);
    return !(m_upper.min(ref.m_upper)<m_lower.max(ref.m_lower));
  }
}

template<typename Ty, bool Prot, class Cmp>
bool interval_domain<Ty, Prot, Cmp>::equals(abstract_domain const &other) const {
  if( type_name()!=other.type_name() )
    return false;
  else {
    interval_domain<Ty, Prot, Cmp> const &ref
      = dynamic_cast<interval_domain<Ty, Prot, Cmp> const &>(other);
    return m_lower==ref.m_lower && m_upper==ref.m_upper;
  }
}



// modifiers :

template<typename Ty, bool Prot, class Cmp>
void interval_domain<Ty, Prot, Cmp>::parse_singleton(std::string const &val) {
  bound tmp = boost::lexical_cast<bound>(val);
  if( tmp.is_infinity() )
    throw SYSTEM_ERROR(make_error(domain_error::empty_domain),
                       "attempt to set as singleton to infinity");
  m_lower = tmp;
  m_upper = tmp;
}


template<typename Ty, bool Prot, class Cmp>
void interval_domain<Ty, Prot, Cmp>::parse_lower(std::string const &val) {
  bound tmp = boost::lexical_cast<bound>(val);
  if( m_upper<tmp || 
      ( m_upper==tmp && m_upper.is_infinity() ) )
    throw SYSTEM_ERROR(make_error(domain_error::empty_domain),
                       "attempt to set lower bound above upper");
  m_lower = tmp;
}

template<typename Ty, bool Prot, class Cmp>
void interval_domain<Ty, Prot, Cmp>::parse_upper(std::string const &val) {
  bound tmp = boost::lexical_cast<bound>(val);
  if( m_lower>tmp || 
      ( m_lower==tmp && m_lower.is_infinity() ) )
    throw SYSTEM_ERROR(make_error(domain_error::empty_domain),
                       "attempt to set upper bound below lower");
  m_upper = tmp;
}

template<typename Ty, bool Prot, class Cmp>
bool interval_domain<Ty, Prot, Cmp>::restrict_with
(interval_domain<Ty, Prot, Cmp>::bound const &lo,
 interval_domain<Ty, Prot, Cmp>::bound const &hi) {
  bound const &low = m_lower.max(lo);
  bound const &up = m_upper.min(hi);
  if( up<low )
    throw SYSTEM_ERROR(make_error(domain_error::empty_domain),
                       "intervals do not overlap");
  if(low!=m_lower || up!=m_upper) {
    m_lower = low;
    m_upper = up;
    return true;
  }
  return false;
}


template<typename Ty, bool Prot, class Cmp>
bool interval_domain<Ty, Prot, Cmp>::restrict_with
(abstract_domain const &other) {
  if( type_name()!=other.type_name() )
    throw SYSTEM_ERROR(make_error(domain_error::incompatible_types));
  else {
    interval_domain<Ty, Prot, Cmp> const &ref
      = dynamic_cast<interval_domain<Ty, Prot, Cmp> const &>(other);
    return restrict_with(ref.lower_bound(), ref.upper_bound());
  }
  return false;
}


#endif 
