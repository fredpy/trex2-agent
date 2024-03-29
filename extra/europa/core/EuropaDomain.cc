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

#include <trex/europa/EuropaEntity.hh>
#include "private/EuropaDomain.hh"
#include <trex/europa/bits/europa_convert.hh>

using namespace TREX::europa;
using namespace TREX;

namespace tr=TREX::transaction;
using TREX::utils::Symbol;


/*
 * class TREX::europa::EuropaEntity
 */

// statics 

Symbol const EuropaEntity::type_name("europa_object");

/*
 * class TREX::europa::details::EuropaDomain
 */

// statics 

Symbol const details::EuropaDomain::type_name("europa_domain");

EUROPA::Domain *details::EuropaDomain::safe_copy(EUROPA::Domain *dom) {
  if( NULL!=dom ) 
    return dom->copy();
  return NULL;
}

// structors

details::EuropaDomain::EuropaDomain(EUROPA::Domain const &dom)
  :tr::DomainBase(type_name), m_dom(dom.copy()) {}

details::EuropaDomain::EuropaDomain(EuropaDomain const &other)
  :tr::DomainBase(type_name), m_dom(safe_copy(other.m_dom)) {}

details::EuropaDomain::~EuropaDomain() {
  if( NULL!=m_dom )
    delete m_dom;
}

// observers

bool details::EuropaDomain::isFull() const {
  EUROPA::DataTypeId const &type = europaDomain().getDataType();
  return type->baseDomain()==europaDomain();
}

bool details::EuropaDomain::intersect(DomainBase const &other) const {
  try {  // this try/catch is not very efficient ... should find a way to avoid this
    europa_domain visit(europaDomain().getDataType());
    other.accept(visit);
    
    return europaDomain().intersects(visit.domain());
  } catch(...) {
    return false;
  }
}

bool details::EuropaDomain::equals(DomainBase const &other) const {
  try { // this try/catch is not very efficient ... should find a way to avoid this
    europa_domain visit(europaDomain().getDataType());
    other.accept(visit);
    
    return europaDomain()==visit.domain();
  } catch(...) {
    return false;
  }
}

boost::any details::EuropaDomain::singleton() const {
  // copy shoud not be too costful as I should have already checked
  // that the domain is a singleton
  std::unique_ptr<tr::DomainBase> tmp(copy());
  return tmp->getSingleton();
}

std::string details::EuropaDomain::stringSingleton() const {
  // copy shoud not be too costful as I should have already checked
  // that the domain is a singleton
  std::unique_ptr<tr::DomainBase> tmp(copy());
  return tmp->getStringSingleton();
}

tr::DomainBase *details::EuropaDomain::copy() const {
  // copy implies a conversion to a real TREX representation
  // this can be costfull but at least won't happen until explicitely
  // copy the domain
  return trex_domain(europaDomain());
}

boost::property_tree::ptree details::EuropaDomain::build_tree() const {
  std::unique_ptr<tr::DomainBase> tmp(copy());
  return tmp->build_tree();
}


//std::ostream &details::EuropaDomain::toXml(std::ostream &out, size_t tabs) const {
//  // this copy can be costfull but guarantees that the output can be
//  // parsed by TREX
//  std::unique_ptr<tr::DomainBase> tmp(copy());
//  return tmp->toXml(out, tabs);
//}

std::ostream &details::EuropaDomain::print_domain(std::ostream &out) const {
  // this copy is costfiull and we may want to avoid this
  // (so we see that the output is from europa)
  std::unique_ptr<tr::DomainBase> tmp(copy());
  return out<<(*tmp);
}

// modifiers 

tr::DomainBase &details::EuropaDomain::restrictWith(tr::DomainBase const &other) {
  europa_domain visit(m_dom);
  other.accept(visit);
  return *this;
}



namespace {

  // declare EuropaEntity domain
  tr::DomainBase::xml_factory::declare<EuropaEntity> decl(EuropaEntity::type_name);

} // ::
