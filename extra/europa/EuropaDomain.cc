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
#include "EuropaDomain.hh"
#include "Assembly.hh"

#include <trex/domain/BooleanDomain.hh>
#include <trex/domain/IntegerDomain.hh>
#include <trex/domain/FloatDomain.hh>
#include <trex/domain/StringDomain.hh>
#include <trex/domain/EnumDomain.hh>

#include "bits/europa_convert.hh"

#include <memory>

using namespace TREX::europa;
using namespace TREX::europa::details;
using namespace TREX::transaction;
using namespace TREX::utils;

DomainBase *TREX::europa::details::trex_domain(EUROPA::Domain const &dom) {
  EUROPA::DataTypeId const &type(dom.getDataType());
  std::auto_ptr<DomainBase> result;
  
  // Hard coded approach for now
  if( type->isBool() ) {
    // boolean 
    if( dom.isSingleton() )
      result.reset(new BooleanDomain(dom.getSingletonValue()==0.0));
    else 
      result.reset(new BooleanDomain());
  } else if( type->isNumeric() ) {
    EUROPA::edouble e_lb, e_ub;
    
    dom.getBounds(e_lb, e_ub);
    if( 1.0==type->minDelta() ) {
      EUROPA::eint i_lb(e_lb), i_ub(e_ub);
      IntegerDomain::bound t_lb = IntegerDomain::minus_inf,
	t_ub = IntegerDomain::plus_inf;

      if( std::numeric_limits<EUROPA::eint>::minus_infinity()<i_lb )
	t_lb = EUROPA::cast_basis(i_lb);
      if( std::numeric_limits<EUROPA::eint>::infinity()>i_ub )
	t_ub = EUROPA::cast_basis(i_ub);
      result.reset(new IntegerDomain(t_lb, t_ub));
    } else {
      FloatDomain::bound t_lb = FloatDomain::minus_inf,
	t_ub = FloatDomain::plus_inf;

      if( std::numeric_limits<EUROPA::edouble>::minus_infinity()<e_lb )
	t_lb = EUROPA::cast_basis(e_lb);
      if( std::numeric_limits<EUROPA::edouble>::infinity()>e_ub )
	t_ub = EUROPA::cast_basis(e_ub);
      result.reset(new FloatDomain(t_lb, t_ub));
    }
  } else if( type->isString() ) {
    std::list<EUROPA::edouble> values;
    StringDomain tmp;
    
    dom.getValues(values);
    for(std::list<EUROPA::edouble>::const_iterator i=values.begin();
	values.end()!=i; ++i) 
      tmp.add(type->toString(*i));
    result.reset(tmp.copy());
  } else if( type->isEntity() ) {
    std::list<EUROPA::edouble> values;
    EuropaEntity tmp;
    
    dom.getValues(values);
    for(std::list<EUROPA::edouble>::const_iterator i=values.begin();
	values.end()!=i; ++i) 
      tmp.add(type->toString(*i));
    result.reset(tmp.copy());
  } else if( type->isSymbolic() ) {
    std::list<EUROPA::edouble> values;
    TREX::transaction::EnumDomain tmp;
    
    dom.getValues(values);
    for(std::list<EUROPA::edouble>::const_iterator i=values.begin();
	values.end()!=i; ++i) 
      tmp.add(type->toString(*i));
    result.reset(tmp.copy());
  } else {
    throw EuropaException("Don't know how to convert europa type "+
			  type->getName().toString()+
			  " to an equivalent TREX doomain");
  }
  return result.release();
}


/*
 * class TREX::europa::EuropaEntity
 */
// static

Symbol const EuropaEntity::type_name("europa_object");

namespace {

  // declare EuropaEntity domain
  DomainBase::xml_factory::declare<EuropaEntity> decl(EuropaEntity::type_name);

} // ::


/*
 * class TREX::europa::EuropaDomain
 */
// statics 

Symbol const EuropaDomain::type_name("europa_domain");

EUROPA::Domain *EuropaDomain::safe_copy(EUROPA::Domain *dom) {
  if( NULL!=dom ) 
    return dom->copy();
  return NULL;
}

// structors 

EuropaDomain::EuropaDomain(EUROPA::Domain const &dom) 
  :DomainBase(type_name), m_dom(dom.copy()) {}
 
EuropaDomain::EuropaDomain(EuropaDomain const &other) 
  :DomainBase(other), m_dom(safe_copy(other.m_dom)) {}

EuropaDomain::~EuropaDomain() {
  if( NULL!=m_dom ) 
    delete m_dom;
}

// observers 

bool EuropaDomain::isFull() const {
  EUROPA::DataTypeId const &type = europaDomain().getDataType();
  return type->baseDomain()==europaDomain();
}

boost::any EuropaDomain::singleton() const {
  std::auto_ptr<DomainBase> tmp(copy());
  return tmp->getSingleton();  
}

std::string EuropaDomain::stringSingleton() const {
  std::auto_ptr<DomainBase> tmp(copy());
  return tmp->getStringSingleton();
}

bool EuropaDomain::intersect(DomainBase const &other) const {
  try {
    europa_domain visit(europaDomain().getDataType());
    other.accept(visit);

    return europaDomain().intersects(visit.domain());
  } catch(...) {
    return false;
  }
}

bool EuropaDomain::equals(DomainBase const &other) const {
  try {
    europa_domain visit(europaDomain().getDataType());
    other.accept(visit);

    return europaDomain()==visit.domain();
  } catch(...) {
    return false;
  }
}

DomainBase *EuropaDomain::copy() const {
  // The EuropaDomain is just a proxy 
  // => convert the copy into a real TREX domain 
  return trex_domain(europaDomain());
}

std::ostream &EuropaDomain::toXml(std::ostream &out, size_t tabs) const {
  std::auto_ptr<DomainBase> tmp(copy());
  return tmp->toXml(out, tabs);
}

std::ostream &EuropaDomain::print_domain(std::ostream &out) const {
  std::auto_ptr<DomainBase> tmp(copy());
  return out<<*tmp;
}

// modifers 

DomainBase &EuropaDomain::restrictWith(DomainBase const &other) {
  europa_domain visit(m_dom);
  other.accept(visit);
  return *this;
}

/*
 * class TREX::europa::details::europa_domain
 */ 

// callbacks

void europa_domain::visit(BasicEnumerated const *dom) {
  if( m_dom->isEnumerated() ) {
    std::list<EUROPA::edouble> values;
    
    for(size_t i=0; i<dom->getSize(); ++i) {
      EUROPA::edouble val = m_type->createValue(dom->getStringValue(i));

      if( m_dom->isMember(val) )
	values.push_back(val);
    }
    if( values.empty() )
      throw EmptyDomain(*dom, "EUROPA enumerated domain "+m_dom->toString()+"became empty.");
    m_dom->empty();
    m_dom->insert(values);
  } else 
    throw DomainAccess(*dom, "EUROPA domain is not enumerated.");
}

void europa_domain::visit(BasicInterval const *dom) {
  if( m_dom->isInterval() ) {
    std::auto_ptr<EUROPA::Domain> tmp(m_dom->copy());
    std::string lo = dom->getStringLower(), hi = dom->getStringUpper();


    debugMsg("trex:assign", "Attempting to intersect "<<m_dom->toString()
	     <<" with ["<<lo<<", "<<hi<<"]");
    EUROPA::edouble elo = m_type->createValue(lo), ehi = m_type->createValue(hi);
    debugMsg("trex:assign", "  - converted ["<<lo<<", "<<hi
	     <<"] into europa "<<m_type->getNameString()<<" ["<<elo<<", "<<ehi<<"]");
    tmp->intersect(elo, ehi);
    debugMsg("trex:assign", "  - intersection is "<<tmp->toString());
    if( tmp->isEmpty() )
      throw EmptyDomain(*dom, "EUROPA interval domain "+m_dom->toString()+" became empty.");
    m_dom->intersect(*tmp);
  } else 
    throw DomainAccess(*dom, "EUROPA domain is not an interval.");
}

void europa_domain::visit(DomainBase const *dom, bool) {
  Symbol const &type = dom->getTypeName();
  
  if( EuropaDomain::type_name==type ) {
    EuropaDomain const &dd = dynamic_cast<EuropaDomain const &>(*dom);
    
    if( !m_dom->intersect(dd.europaDomain()) )
      throw EmptyDomain(*dom, "EUROPA domain "+m_dom->toString()+" became empty.");
  } else 
    throw DomainAccess(*dom, "Don't know how to convert domain "
		       +type.str()+" for EUROPA");
}
