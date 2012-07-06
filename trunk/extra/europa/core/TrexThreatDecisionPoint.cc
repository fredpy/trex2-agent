#include "private/TrexThreatDecisionPoint.hh"

# include <trex/europa/Assembly.hh>

# include <PLASMA/Token.hh>
# include <PLASMA/TokenVariable.hh>



using namespace  TREX::europa;
using EUROPA::ObjectId;
using EUROPA::TokenId;

/*
 * class TREX::europa::TrexThreatDecisionPoint
 */ 

// structors

TrexThreatDecisionPoint::TrexThreatDecisionPoint
  (EUROPA::DbClientId const &client, EUROPA::TokenId const &tokenToOrder,
   EUROPA::TiXmlElement const &configData, EUROPA::LabelStr const &explanation)
  :EUROPA::SOLVERS::ThreatDecisionPoint(client, tokenToOrder, 
					configData, explanation) {
  if( !client->isGlobalVariable(Assembly::CLOCK_VAR) )
    throw EuropaException("Unable to find clock variable \""
			  +Assembly::CLOCK_VAR.toString()+"\"");
  m_clock = client->getGlobalVariable(Assembly::CLOCK_VAR);
}

// manipulators

void TrexThreatDecisionPoint::handleInitialize() {
  ThreatDecisionPoint::handleInitialize();
  
  if( m_tokenToOrder->start()->lastDomain().getUpperBound()>=now() ) { 
    EUROPA::eint min_dur = m_tokenToOrder->duration()->lastDomain().getLowerBound();
    // Prune all the solutions taht would put this token in the past
    std::vector<std::pair<ObjectId, std::pair<TokenId, TokenId> > >::iterator i = m_choices.begin();
    while( m_choices.end()!=i ) {
      if( m_tokenToOrder==i->second.first ) {
	if( m_tokenToOrder!=i->second.second ) {
	  // m_tokenToOrder inserted before i->second.second
	  EUROPA::eint max_start = i->second.second->start()->lastDomain().getUpperBound();
	  if( max_start<=std::numeric_limits<EUROPA::eint>::infinity() ) {
	    max_start = max_start-min_dur;
	    if( max_start < now() ) {
	      debugMsg("trex:threat", "remove (*"<<m_tokenToOrder->getKey()<<"*<"
		       <<i->second.second->getKey()<<")\n\tthe latter start="
		       <<i->second.second->start()->lastDomain().toString()
		       <<" is before "<<now());
	      i = m_choices.erase(i);
	      continue;
	    }
	  }
	} 
      } /* 
	   This code is no good ... need to tinker it but for now 
	   we disable it
	   
	   else {
	   // m_tokenToOrder is inserted after i->second.first 
	   // check that i->second.first ends after now()
	   if( i->second.first->end()->lastDomain().getUpperBound()<now() ) {
	   debugMsg("trex:threat", "remove ("<<i->second.first->getKey()<<"<*"
	   <<m_tokenToOrder->getKey()<<"*)\n\tthe former end="
	   <<i->second.second->end()->lastDomain().toString()
	   <<" is before "<<now());
	   i = m_choices.erase(i);
	   continue;
	   }	
	   }
	*/
      debugMsg("trex:threat", "keeping choice ("<<i->second.first->getKey()<<"<"
	       <<i->second.second->getKey()
	       <<")\n\t- first from : "<<i->second.first->start()->lastDomain().toString()
	       <<" to "<<i->second.first->end()->lastDomain().toString()
	       <<")\n\t- second from : "<<i->second.second->start()->lastDomain().toString()
	       <<" to "<<i->second.second->end()->lastDomain().toString());
      ++i;
    }
  } else {
    debugMsg("trex:threat", m_tokenToOrder->getKey()<<" start="
	     <<m_tokenToOrder->start()->lastDomain().toString()
	     <<" is before "<<now()<<"\n\tdo keep its options open.");
    // Maybe we should exclude all choices that do not put start 
    // in the past ? ... it should have been already done by the 
    // other guy
  }
  m_choiceCount = m_choices.size();
  debugMsg("trex:threat", "Number of choices for "<<m_tokenToOrder->getKey()<<": "<<m_choiceCount);
}

// observers

EUROPA::eint TrexThreatDecisionPoint::now() const {
  return m_clock->lastDomain().getLowerBound();
}

