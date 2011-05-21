/** @file "Goal.cc"
 * @brief Goal implementation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup transaction
 */
#include "Goal.hh"

using namespace TREX::utils;
using namespace TREX::transaction;

namespace {
  /** @brief Goal declaration for XML parsing
   * @ingroup transaction
   */
  Predicate::xml_factory::declare<Goal> decl_goal("Goal");

}

/*
 * class TREX::transaction::Goal 
 */
// statics :

IntegerDomain const Goal::s_dateDomain;
IntegerDomain const Goal::s_durationDomain(1, IntegerDomain::plus_inf);
Symbol const Goal::s_startName("start");
Symbol const Goal::s_durationName("duration");
Symbol const Goal::s_endName("end"); 

// structors :

Goal::Goal(Goal const &other)
  :Predicate(other), m_start(other.m_start), 
   m_duration(other.m_duration), m_end(other.m_end) {}

Goal::Goal(Symbol const &object, Symbol const &pred)
  :Predicate(object, pred), m_start(s_startName, s_dateDomain), 
   m_duration(s_durationName, s_durationDomain), 
   m_end(s_endName, s_dateDomain) {}

Goal::Goal(rapidxml::xml_node<> const &node)
  :Predicate(node), m_start(s_startName, s_dateDomain), 
   m_duration(s_durationName, s_durationDomain), 
   m_end(s_endName, s_dateDomain) {
  iterator iStart = find(s_startName), 
    iDuration, iEnd;
  iterator const endi = end();
  IntegerDomain dStart(s_dateDomain), dDuration(s_durationDomain),
    dEnd(s_dateDomain);
  if( endi!=iStart ) {
    try {
      dStart.restrictWith(iStart->second.domain());
    } catch( EmptyDomain const &e ) {
      throw PredicateException(std::string("Goal: start domain : ")+e.what());
    }
    remove(iStart);
  }
  iDuration = find(s_durationName);
  if( endi!=iDuration ) {
    try {
      dDuration.restrictWith(iDuration->second.domain());
    } catch( EmptyDomain const &e ) {
      throw PredicateException(std::string("Goal: duration domain : ")+e.what());
    }
    remove(iDuration);
  }
  iEnd = find(s_endName);
  if( endi!=iEnd ) {
    try {
      dEnd.restrictWith(iEnd->second.domain());
    } catch( EmptyDomain const &e ) {
      throw PredicateException(std::string("Goal: end domain : ")+e.what());
    }
    remove(iEnd);
  }
  restrictTime(dStart, dDuration, dEnd);
}

// Modifiers :

void Goal::restrictAttribute(Variable const &var) {
  Symbol const &id = var.name();
  // Handle specific temporal attributes
  if( s_startName==id )
    restrictStart(var.typedDomain<IntegerDomain>());
  else if( s_durationName==id ) 
    restrictDuration(var.typedDomain<IntegerDomain>());
  else if( s_durationName==id ) 
    restrictEnd(var.typedDomain<IntegerDomain>());
  else 
    Predicate::restrictAttribute(var);
}

void Goal::restrictTime(IntegerDomain const &s,
			IntegerDomain const &d,
			IntegerDomain const &e) {
  IntegerDomain::bound sLo, sHi, dLo, dHi, eLo, eHi;
  IntegerDomain::bound mLo, mHi, aLo, aHi;

  if( !( m_start.domain().intersect(s) && 
	 m_duration.domain().intersect(d) &&
	 m_end.domain().intersect(e) ) )
    throw PredicateException("Invalid time constraint on Goal");
  // Compute the new domains
  m_start.typedDomain<IntegerDomain>().getBounds(sLo, sHi);
  
  s.getBounds(aLo, aHi);
  // s = s inter start
  sLo = sLo.max(aLo);
  sHi = sHi.min(aHi);

  m_duration.typedDomain<IntegerDomain>().getBounds(dLo, dHi);
  d.getBounds(aLo, aHi);
  // d = d inter duration 
  dLo = dLo.max(aLo);
  dHi = dHi.min(aHi);

  m_end.typedDomain<IntegerDomain>().getBounds(eLo, eHi);
  e.getBounds(aLo, aHi);
  // e = e inter end
  eLo = eLo.max(aLo);
  eHi = eHi.min(aHi);
 
  // propagate end time 
  // e = e inter s+d
  eHi = eHi.min(sHi.plus(dHi, eHi));
  eLo = eLo.max(sLo.plus(dLo, eLo));

  if( eLo>eHi )
    throw PredicateException("end time is empty after propagation");

  // propagate start time 
  // s = s inter e-d
  sHi = sHi.min(eHi.minus(dLo, sHi));
  sLo = sLo.max(eLo.minus(dHi, sLo));

  if( sLo>sHi )
    throw PredicateException("start time is empty after propagation");

  // propagate duration 
  // d = d inter e-s
  dHi = dHi.min(eHi.minus(sLo, dHi));
  dLo = dLo.max(eLo.minus(sHi, dLo));

  if( dLo>dHi )
    throw PredicateException("duration is empty after propagation");
  try {
    // start = start inter s
    m_start.restrict(IntegerDomain(sLo, sHi));
  } catch( EmptyDomain const &e) {
    // this should not happen
    throw PredicateException(std::string("Goal: start progation: ")+e.what());
  }
  try {
    // duration = duration inter d
    m_duration.restrict(IntegerDomain(dLo, dHi));
  } catch( EmptyDomain const &e) {
    // this should not happen
    throw PredicateException(std::string("Goal: duration progation: ")+e.what());
  }
  try {
    // end = end inter e
    m_end.restrict(IntegerDomain(eLo, eHi));
  } catch( EmptyDomain const &e) {
    // this should not happen
    throw PredicateException(std::string("Goal: end progation: ")+e.what());
  }
}

bool Goal::startsAfter(TICK date, TICK delay) {
  IntegerDomain test_window(date+delay, IntegerDomain::plus_inf),
    cstr_window(date, IntegerDomain::plus_inf);
  if( m_start.typedDomain<IntegerDomain>().intersect(test_window) ) {
    restrictTime(cstr_window, s_durationDomain, s_dateDomain);
    return true;
  }
  return false;
}

// Observers :

Variable const &Goal::getAttribute(Symbol const &name) const {
  if( s_startName==name )
    return m_start;
  else if( s_durationName==name ) 
    return m_duration;
  else if( s_endName==name ) 
    return m_end;
  else 
    return Predicate::getAttribute(name);
}

IntegerDomain const &Goal::getStart() const {
  m_start.typedDomain<IntegerDomain>();
}

IntegerDomain const &Goal::getDuration() const {
  m_duration.typedDomain<IntegerDomain>();
}

IntegerDomain const &Goal::getEnd() const {
  m_end.typedDomain<IntegerDomain>();
}


void Goal::listAttributes(std::list<TREX::utils::Symbol> &attr,
			  bool all) const {
  if( all || !m_start.domain().isFull() )
    attr.push_back(s_startName);
  if( all || !m_duration.domain().equals(s_durationDomain) )
    attr.push_back(s_durationName);
  if( all || !m_end.domain().isFull() )
    attr.push_back(s_endName);
  Predicate::listAttributes(attr, all);
}
  
Symbol const &Goal::getPredTag() const {
  static Symbol const name("Goal");
  return name;
}
