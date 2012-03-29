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
#include "bits/europa_convert.hh"


#include <trex/europa/EuropaException.hh>
#include "EuropaEntity.hh"
#include "private/EuropaDomain.hh"

#include <trex/domain/BooleanDomain.hh>
#include <trex/domain/IntegerDomain.hh>
#include <trex/domain/FloatDomain.hh>
#include <trex/domain/StringDomain.hh>
#include <trex/domain/EnumDomain.hh>

#include <PLASMA/Domains.hh>

#include <memory>

using namespace TREX::europa;
using namespace TREX::transaction;
using namespace TREX::utils;

DomainBase *TREX::europa::details::trex_domain(EUROPA::Domain const &dom) {
  EUROPA::DataTypeId const &type(dom.getDataType());
  std::auto_ptr<DomainBase> result;

  // There should be a less hard-coded way to do it ...
  // this current implementation is not really open to the addition of
  // new types
  if( type->isBool() ) {
    // boolean 
    if( dom.isSingleton() )
      result.reset(new BooleanDomain(0.0!=dom.getSingletonValue()));
    else
      result.reset(new BooleanDomain());
  } else if( type->isNumeric() ) {
    EUROPA::edouble e_lb, e_ub;
    
    if( dom.isSingleton() ) 
      e_lb = e_ub = dom.getSingletonValue();
    else 
      dom.getBounds(e_lb, e_ub);
    
    if( 1.0==type->minDelta() ) {
      // integer
      EUROPA::eint i_lb(e_lb), i_ub(e_ub);
      IntegerDomain::bound t_lb = IntegerDomain::minus_inf,
	t_ub = IntegerDomain::plus_inf;

      // Assign the bounds only if they are not infinity
      if( std::numeric_limits<EUROPA::eint>::minus_infinity()<i_lb )
	t_lb = EUROPA::cast_basis(i_lb);
      if( std::numeric_limits<EUROPA::eint>::infinity()>i_ub )
	t_ub = EUROPA::cast_basis(i_ub);
      result.reset(new IntegerDomain(t_lb, t_ub));
    } else {
      // should be float 
      FloatDomain::bound t_lb = FloatDomain::minus_inf,
	t_ub = FloatDomain::plus_inf;

      // Assign the bounds only if they are not infinity
      if( std::numeric_limits<EUROPA::edouble>::minus_infinity()<e_lb )
	t_lb = EUROPA::cast_basis(e_lb);
      if( std::numeric_limits<EUROPA::edouble>::infinity()>e_ub )
	t_ub = EUROPA::cast_basis(e_ub);
      result.reset(new FloatDomain(t_lb, t_ub));
    }
  } else {
    // It should be an enumerated domain 
    std::list<EUROPA::edouble> values;
    BasicEnumerated *tmp = NULL;
    
    if( type->isString() ) {
      // A bag of strings
      tmp = new StringDomain();
    } else if( type->isEntity() ) {
      // A set of europa objects
      tmp = new EuropaEntity();
    } else if( type->isSymbolic() ) {
      // A set of symbols/an enum
      tmp = new EnumDomain();
    } else {
      // don't know what it is
      throw EuropaException("Don't know how to convert Europa type "+
			    type->getName().toString()+
			    " to an equivalent TREX domain.");
    }
    // If I reached this point tmp should be an EnumeratedDomain
    result.reset(tmp);
    dom.getValues(values);
    for(std::list<EUROPA::edouble>::const_iterator i=values.begin();
	values.end()!=i; ++i) 
      tmp->addTextValue(type->toString(*i));
  }
  return result.release(); // release the resulting domain
}

/*
 * class TREX::europa::details::europa_domain 
 */

// manipulators

void details::europa_domain::visit(BasicEnumerated const *dom) {
  if( m_dom->isEnumerated() ) {
    std::list<EUROPA::edouble> values;
    
    // convert all the values of *dom in their europa form
    for(size_t i=0; i<dom->getSize(); ++i) {
      EUROPA::edouble val = m_type->createValue(dom->getStringValue(i));
      
      if( m_dom->isMember(val) ) 
	// insert only the values that are already in the europa domain
	values.push_back(val);
    }
    if( values.empty() ) // the intersection is empty
      throw EmptyDomain(*dom, "Europa enumerated domain "+m_dom->toString()+
			" became empty.");
    // Apply the intersection
    EUROPA::EnumeratedDomain tmp(m_type, values);
    m_dom->intersect(tmp);
  } else 
    throw DomainAccess(*dom, "Europa domain "+m_dom->toString()+" is not enumerated.");
}

void details::europa_domain::visit(BasicInterval const *dom) {
  if( m_dom->isInterval() ) {
    std::auto_ptr<EUROPA::Domain> tmp(m_dom->copy());
    
    if( m_dom->isBool() ) {
      // Handle the boolean special case: we assume here that the T-REX 
      // interval domain is convertible to double
      if( dom->isSingleton() ) {
        double val = dom->getTypedSingleton<double, true>();
        EUROPA::edouble eval(val!=0.0);
        tmp->intersect(eval, eval);
      } else if( !dom->isFull() ) {
        double lo = dom->getTypedLower<double, true>(),
           hi = dom->getTypedUpper<double, true>();
        if( lo>0.0 || hi<0.0 ) {
          EUROPA::eint true_val(1);
          tmp->intersect(true_val, true_val);
        }
      }
    } else {
      std::string lo = dom->getStringLower(), hi = dom->getStringUpper();
      EUROPA::edouble elo = m_type->createValue(lo),
        ehi = m_type->createValue(hi);

      tmp->intersect(elo, ehi);
    }
    if( tmp->isEmpty() )
      throw EmptyDomain(*dom, "Europa Interval domain "+m_dom->toString()
			+" became empty.");
    m_dom->intersect(*tmp);
  } else 
    throw DomainAccess(*dom, "Europa domain "+m_dom->toString()+" is not an interval.");
}

void details::europa_domain::visit(DomainBase const *dom, bool) {
  Symbol const &type = dom->getTypeName();

  if( EuropaDomain::type_name==type ) {
    // EuropaDomain has a europa Domain directly available
    EuropaDomain const &ed = dynamic_cast<EuropaDomain const &>(*dom);
    
    if( !m_dom->intersect(ed.europaDomain()) ) 
      throw EmptyDomain(*dom, "EUROPA domain "+m_dom->toString()+" became empty.");    
  } else 
    // I have no way to know at this stage
    throw DomainAccess(*dom, "Don't know how to convert domains "+
		       type.str()+" for Europa.");
}


