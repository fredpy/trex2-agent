#include <ctime>

#include "TransactionLogger.hh"

using namespace TREX::transaction;
using namespace TREX::utils;

/*
 * class TREX::transaction::TransactionLogger
 */ 
// structors 

TransactionLogger::TransactionLogger()
  :m_inHeader(true), m_empty(true), m_hasData(false) {}

TransactionLogger::~TransactionLogger() {
  endFile();
}

// Modifiers 

void TransactionLogger::startFile(std::string fileName) {
  time_t cur_date;

  fileName = m_log->file_name(fileName);
  m_logFile.open(fileName.c_str());
  
  time(&cur_date);
  char *str_date = ctime(&cur_date);
  str_date[strlen(str_date)-1] = '\0'; // remove trailing \n
  m_logFile<<"<?xml version=\"1.0\" standalone=\"no\"?>\n\n"
	   <<"<Log date=\""<<str_date<<"\">\n"
	   <<" <Declare>"<<std::endl;
}

void TransactionLogger::endFile() {
  if( m_logFile.is_open() ) {
    if( !m_empty ) 
      m_logFile<<" </Tick>\n";
    m_logFile<<"</Log>"<<std::endl;
    m_logFile.close();
  }
}

void TransactionLogger::declInternal(Symbol const &timeline,
				     Symbol const &owner) {
  if( m_inHeader ) {
    std::map<Symbol, Symbol>::value_type to_ins(timeline, owner);
    if( !m_internals.insert(to_ins).second ) {
      m_log->syslog("log")<<"WARN: timeline \""<<timeline
			  <<"\" multiply declared as internal.\n";
    }
  }
}

void TransactionLogger::declExternal(Symbol const &timeline,
				     Symbol const &owner) {
  if( m_inHeader ) {
    m_externals[owner].insert(timeline);
  }
}

void TransactionLogger::endHeader(TICK init, std::string const &fileName) {
  if( m_inHeader ) {
    if( !( m_internals.empty() && m_externals.empty() ) ) {
      std::map< Symbol, std::list<Symbol> > intDecl;
      std::map<Symbol, Symbol>::const_iterator i;

      startFile(fileName);
      for(i=m_internals.begin(); m_internals.end()!=i; ++i) 
	intDecl[i->second].push_back(i->first);

      std::map<Symbol, std::set<Symbol> >::const_iterator e;
      std::map< Symbol, std::list<Symbol> >::iterator id;

      for(e=m_externals.begin(); m_externals.end()!=e; ++e) {
	m_logFile<<"  <Timelines reactor=\""<<e->first<<"\">\n";
	id = intDecl.find(e->first);
	if( intDecl.end()!=id ) {
	  while( !id->second.empty() ) {
	    m_logFile<<"   <Internal name=\""<<id->second.front()<<"\"/>\n";
	    id->second.pop_front();
	  }
	  intDecl.erase(id);
	}
	std::set<Symbol>::const_iterator ed;
	for( ed=e->second.begin(); e->second.end()!=ed; ++ed ) 
	  m_logFile<<"   <External name=\""<<*ed<<"\"/>\n";
	m_logFile<<"  </Timelines>\n";
      }
      // declaring remaining reactors that do not have exteranl timelines
      for(id=intDecl.begin(); intDecl.end()!=id; ++id) {
	m_logFile<<"  <Timelines reactor=\""<<id->first<<"\">\n";
	while( !id->second.empty() ) {
	  m_logFile<<"   <Internal name=\""<<id->second.front()<<"\"/>\n";
	  id->second.pop_front();
	}
	m_logFile<<"  </Timelines>\n";
      }
      m_logFile<<" </Declare>"<<std::endl;
    }
    m_inHeader = false;
  }
  newTick(init);
}

void TransactionLogger::newTick(TICK value) {
  m_lastTick = value;
  m_hasData = false;
}

void TransactionLogger::logObservation(Observation const &obs) {
  if( m_logFile.is_open() ) {
    Symbol const &name = obs.object();
    if( m_internals.find(name)!=m_internals.end() ) {
      openTick();
      obs.toXml(m_logFile, 2)<<std::endl;
    }
  }
}

void TransactionLogger::logRequest(Symbol const &reactor, goal_id const &goal) {
  if( m_logFile.is_open() ) {
    std::map<Symbol, std::set<Symbol> >::const_iterator e = m_externals.find(reactor);
    if( m_externals.end()!=e ) {
      openTick();
      m_logFile<<"  <Request from=\""<<reactor<<"\" id=\""<<goal<<"\">\n";
      goal->toXml(m_logFile, 3);
      m_logFile<<"  </Request>"<<std::endl;
    }
  }
}

void TransactionLogger::logRecall(Symbol const &reactor, goal_id const &goal) {
  if( m_logFile.is_open() ) {
    std::map<Symbol, std::set<Symbol> >::const_iterator e = m_externals.find(reactor);
    if( m_externals.end()!=e ) {
      openTick();
      m_logFile<<"  <Recall from=\""<<reactor
	       <<"\" id=\""<<goal<<"\"/>"<<std::endl;
    }
  }
}

void TransactionLogger::openTick() {
  if( !m_hasData ) {
    m_hasData = true;
    if( m_empty )
      m_empty = false;
    else
      m_logFile<<" </Tick>\n";
    m_logFile<<" <Tick value=\""<<m_lastTick<<"\">\n";
  }
}

