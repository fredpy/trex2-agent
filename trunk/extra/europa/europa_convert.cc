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

#include <trex/domain/boolean_domain.hh>
#include <trex/domain/int_domain.hh>
#include <trex/domain/float_domain.hh>
#include <trex/domain/string_domain.hh>
#include <trex/domain/enum_domain.hh>

// include plasma header as system files in order to disable warnings
# define TREX_PP_SYSTEM_FILE <PLASMA/Domains.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/Object.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/PlanDatabase.hh>
# include <trex/europa/bits/system_header.hh>

#include <memory>

using namespace TREX::europa;
using namespace TREX;
using TREX::utils::symbol;

namespace tr=TREX::transaction;

tr::abstract_domain *TREX::europa::details::trex_domain(EUROPA::Domain const &dom) {
  EUROPA::DataTypeId const &type(dom.getDataType());
  UNIQ_PTR<tr::abstract_domain> result;

  // There should be a less hard-coded way to do it ...
  // this current implementation is not really open to the addition of
  // new types
  if( type->isBool() ) {
    // boolean 
    if( dom.isSingleton() )
      result.reset(new tr::boolean_domain(0.0!=dom.getSingletonValue()));
    else
      result.reset(new tr::boolean_domain());
  } else if( type->isNumeric() ) {
    EUROPA::edouble e_lb, e_ub;
    
    if( dom.isSingleton() ) {
      // We need to handle specifically singletons as in europa some singletons 
      // do not have the same lower bound and upper bound (if the lower bound is
      // close enough to the upper bound europa mark it as a singleton)
      e_lb = e_ub = dom.getSingletonValue();
    } else 
      dom.getBounds(e_lb, e_ub);
    
    if( 1.0==type->minDelta() ) {
      // integer
      EUROPA::eint i_lb(e_lb), i_ub(e_ub);
      tr::int_domain::bound t_lb = tr::int_domain::minus_inf,
        t_ub = tr::int_domain::plus_inf;

      // Assign the bounds only if they are not infinity
      if( std::numeric_limits<EUROPA::eint>::minus_infinity()<i_lb )
	t_lb = EUROPA::cast_basis(i_lb);
      if( std::numeric_limits<EUROPA::eint>::infinity()>i_ub )
	t_ub = EUROPA::cast_basis(i_ub);
      result.reset(new tr::int_domain(t_lb, t_ub));
    } else {
      // should be float 
      tr::float_domain::bound t_lb = tr::float_domain::minus_inf,
	t_ub = tr::float_domain::plus_inf;

      // Assign the bounds only if they are not infinity
      if( std::numeric_limits<EUROPA::edouble>::minus_infinity()<e_lb )
	t_lb = EUROPA::cast_basis(e_lb);
      if( std::numeric_limits<EUROPA::edouble>::infinity()>e_ub )
	t_ub = EUROPA::cast_basis(e_ub);
      result.reset(new tr::float_domain(t_lb, t_ub));
    }
  } else {
    std::list<EUROPA::edouble> values;
    tr::basic_enumerated *tmp = NULL;
    
    if( type->isEntity() ) {
      tmp = new EuropaEntity();
      result.reset(tmp);
      dom.getValues(values);
      for(std::list<EUROPA::edouble>::const_iterator i=values.begin();
	  values.end()!=i; ++i) {
	std::string str = type->toString(*i);
	// remove the (id) at the end
	if( str[str.length()-1]==')' ) 
	  str = str.substr(0, str.rfind('('));
	tmp->add_string_value(str);
      }
    } else {
      // It should be an enumerated domain 
      if( type->isString() ) {
	// A bag of strings
	tmp = new tr::string_domain();
      } else if( type->isSymbolic() ) {
	// A set of symbols/an enum
	tmp = new tr::enum_domain();
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
	tmp->add_string_value(type->toString(*i));
    }
  }
  return result.release(); // release the resulting domain
}

/*
 * class TREX::europa::details::europa_domain 
 */

// manipulators

void details::europa_domain::visit(tr::basic_enumerated const *dom) {
  if( m_dom->isEntity() ) {
    // I assume that only objects are entity ... should be fine I think
    EUROPA::ObjectDomain *o_dom = dynamic_cast<EUROPA::ObjectDomain *>(m_dom);
    std::list<EUROPA::edouble> values;
    EUROPA::edouble val;
        
    if( NULL==o_dom )
      throw tr::DomainAccess(*dom, "Europa domain "+m_dom->toString()+" is an entity but not an object.\nDo not know how to handle this.");
    else {
      EUROPA::ObjectId obj; 
      EUROPA::PlanDatabaseId p_db; 
      
      // Need to identify the plan database 
      std::list<EUROPA::ObjectId> objs = o_dom->makeObjectList();
      if( objs.empty() )
        throw tr::DomainAccess(*dom, "Europa domain "+o_dom->toString()+" was empty before the operation");
      else
        p_db = objs.front()->getPlanDatabase();
      objs.clear();
      
      for(size_t i=0; i<dom->size(); ++i) {
        obj = p_db->getObject(dom->element_as_string(i));
        if( obj.isId() && o_dom->isMember(obj) )
          objs.push_back(obj);
      }
      if( objs.empty() ) // the intersection is empty
        throw tr::EmptyDomain(*dom, "Europa object domain "+m_dom->toString()+
                          " became empty.");
      EUROPA::ObjectDomain tmp(m_type, objs);
      if( m_dom->intersect(tmp) )
        *m_updated = true;
    }
    
  } else if( m_dom->isEnumerated() ) {
    std::list<EUROPA::edouble> values;
    
    EUROPA::edouble val;
    
    for(size_t i=0; i<dom->size(); ++i) {
      if( m_dom->convertToMemberValue(dom->element_as_string(i), val) )
        values.push_back(val);
    }
    if( values.empty() ) // the intersection is empty
      throw tr::EmptyDomain(*dom, "Europa enumerated domain "+m_dom->toString()+
			" became empty.");
    // Apply the intersection
    EUROPA::EnumeratedDomain tmp(m_type, values);
    if( m_dom->intersect(tmp) )
      *m_updated = true;
  } else 
    throw tr::DomainAccess(*dom, "Europa domain "+m_dom->toString()+" is not enumerated.");
}

namespace {
  float decimal_places(double val, size_t n_places) {
    double factor = pow(10.0L, n_places);
    return std::floor(val*factor) / factor;
  }
}


void details::europa_domain::visit(tr::basic_interval const *dom) {
  if( m_dom->isInterval() ) {
    UNIQ_PTR<EUROPA::Domain> tmp(m_dom->copy());
    
    if( m_dom->isBool() ) {
      // Handle the boolean special case: we assume here that the T-REX 
      // interval domain is convertible to double
      if( dom->is_singleton() ) {
        double val = dom->get_typed_singleton<double, true>();
        EUROPA::edouble eval(val!=0.0);
        tmp->intersect(eval, eval);
      } else if( !dom->is_full() ) {
        double lo = dom->get_typed_lower<double, true>(),
           hi = dom->get_typed_upper<double, true>();
        if( lo>0.0 || hi<0.0 ) {
          EUROPA::eint true_val(1);
          tmp->intersect(true_val, true_val);
        }
      }
    } else {
      std::string lo = dom->lower_as_string(), hi = dom->upper_as_string();
      EUROPA::edouble elo = m_type->createValue(lo),
        ehi = m_type->createValue(hi);
      
      // float values received by trex are restrainted to use only
      // 8 decimal places
      if( elo>std::numeric_limits<EUROPA::edouble>::minus_infinity() )
        elo = decimal_places(EUROPA::cast_basis(elo), 8);
      if( ehi<std::numeric_limits<EUROPA::edouble>::minus_infinity() )
        ehi = decimal_places(EUROPA::cast_basis(ehi), 8);
      
      tmp->intersect(elo, ehi);
    }
    if( tmp->isEmpty() )
      throw tr::EmptyDomain(*dom, "Europa Interval domain "+m_dom->toString()
			+" became empty.");
    if( m_dom->intersect(*tmp) )
      *m_updated = true;
  } else 
    throw tr::DomainAccess(*dom, "Europa domain "+m_dom->toString()+" is not an interval.");
}

void details::europa_domain::visit(tr::abstract_domain const *dom, bool) {
  symbol const &type = dom->type_name();

  if( EuropaDomain::type_str==type ) {
    // EuropaDomain has a europa Domain directly available
    EuropaDomain const &ed = dynamic_cast<EuropaDomain const &>(*dom);
    
    if( !m_dom->intersects(ed.europaDomain()) )
      throw tr::EmptyDomain(*dom, "EUROPA domain "+m_dom->toString()+" became empty.");
    else if( m_dom->intersect(ed.europaDomain()) )
      *m_updated = true;
  } else 
    // I have no way to know at this stage
    throw tr::DomainAccess(*dom, "Don't know how to convert domains "+
		       type.str()+" for Europa.");
}


