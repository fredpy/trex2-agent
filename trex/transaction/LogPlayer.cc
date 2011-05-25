#include "LogPlayer.hh"
#include <set>

using namespace TREX::transaction;
using namespace TREX::utils;

namespace {

  TeleoReactor::xml_factory::declare<LogPlayer> decl("LogPlayer");
  
} // ::

/*
 * class TREX::transaction::LogPLayer
 */

// structors 

LogPlayer::LogPlayer(TeleoReactor::xml_arg_type arg) 
  :TeleoReactor(arg, false) {
  std::string 
    playedReactor = parse_attr<std::string>(getName().str(), TeleoReactor::xml_factory::node(arg), 
					    "reactor"),
    fileName = getAgentName().str()+".log"; // by default it will play this <agent>.log
  fileName = parse_attr<std::string>(fileName, TeleoReactor::xml_factory::node(arg), "file");
  bool found;
  fileName = manager().use(fileName, found);

  if( !found ) {
    syslog()<<"Unable to locate the log file \""<<fileName<<"\".";
    throw ReactorException(*this, "Unable to load specified transaction log file.");
  }
  rapidxml::file<> file(fileName.c_str());
  rapidxml::xml_document<> doc;
  doc.parse<0>(file.data());
  rapidxml::xml_node<> *root = doc.first_node();
  if( NULL==root ) {
    syslog()<<"Transaction log \""<<fileName<<"\" is empty.";    
    throw ReactorException(*this, "Invalid transaction file.");
  }
  loadTransactions(*root, playedReactor);
}

LogPlayer::~LogPlayer() {
  m_obsLog.clear();
  m_requestLog.clear();
  m_recallLog.clear();
}

// modifiers :

void LogPlayer::loadTransactions(rapidxml::xml_node<> &node, std::string const &reactor) {
  // First look for the definition of this reactor
  ext_iterator log(node), it;

  if( !log.valid() ) {
    // Log file is an empty file => Xml error
    syslog()<<"Transaction log file has no sub-node";
    throw XmlError(node, "Empty log file");
  }
  it = log.find_tag("Declare");
  if( !it.valid() ) {
    // Missing Declare tag => Xml error
    syslog()<<"Transaction log file do not have reactors declaration.";
    throw XmlError(node, "Sub node <Declare> was not found");
  }
  it = ext_iterator(*it);
  if( !it.valid() ){
    // Empty Declare tag => Xml error
    syslog()<<"Transaction log file have an empty reactors declaration.";
    throw XmlError(node, "Sub node <Declare> is empty");
  }
  for(it = it.find_tag("Timelines"); 
      it.valid() && reactor!=parse_attr<std::string>(*it, "reactor"); it = it.next("Timelines"));
  if( !it.valid() ) {
    // reactor not found => Reactor Error
    syslog()<<"Unable to find definition of reactor \""<<reactor<<"\".";
    throw ReactorException(*this, "Unable to find reactor declaration");
  }
  std::set<Symbol> internals, externals;

  for(it = ext_iterator(*it); it.valid(); ++it) {
    Symbol tl;
    if( is_tag(*it, "External") ) {
      tl = parse_attr<Symbol>(*it, "name");
      if( externals.insert(tl).second )
	use(tl);
    } else if( is_tag(*it, "Internal") ) {
      tl = parse_attr<Symbol>(*it, "name");
      if( internals.insert(tl).second )
	provide(tl);      
    }
  }
  if( internals.empty() && externals.empty() ) {
    // no timelines => warn and skip
    syslog()<<"WARNING: Reactor \""<<reactor<<"\" has no timeline";
  } else {
    std::map<std::string, goal_id> goals;
    Symbol tl;
    for(log = log.find_tag("Tick"); log.valid(); log = log.next("Tick")) {
      TICK current = parse_attr<TICK>(*log, "value");
      it = ext_iterator(*log);
      for( ; it.valid(); ++it) {
	if( is_tag(*it, "Observation") ) {
	  Symbol tl = parse_attr<Symbol>(*it, "on");
	  // check timeline first to avoid to parse superfluous observations
	  if( internals.find(tl)!=internals.end() ) {
	    Observation obs(*it);
	    m_obsLog.insert(std::make_pair(current, obs));
	  }
	} else if( is_tag(*it, "Request") ) {
	  if( reactor==parse_attr<std::string>(*it, "from") ) {
	    std::string id = parse_attr<std::string>(*it, "id");
	    ext_iterator gxml(*it);
	    gxml = gxml.find_tag("Goal");
	    if( gxml.valid() ) {
	      goal_id g(new Goal(*gxml));
	      m_requestLog.insert(std::make_pair(current, g));
	      goals[id] = g;
	    } else 
	      syslog()<<"WARNING: empty request ("<<id<<").";
	  }
	} else if( is_tag(*it, "Recall") ) {
	  if( reactor==parse_attr<std::string>(*it, "from") ) {
	    std::string id = parse_attr<std::string>(*it, "id");
	    std::map<std::string, goal_id>::iterator g = goals.find(id);
	    if( goals.end()==g ) 
	      syslog()<<"WARNING : logged recall has an unknown id ("<<id<<")";
	    else {
	      m_recallLog.insert(std::make_pair(current, g->second));
	      goals.erase(g);
	    }
	  }
	}
      }
    }    
  }
}

// TREX callbacks

void LogPlayer::handleInit() {
  m_nextObs = m_obsLog.lower_bound(getCurrentTick());
  m_nextRequest = m_requestLog.lower_bound(getCurrentTick());
  m_nextRecall = m_recallLog.lower_bound(getCurrentTick());
}

void LogPlayer::handleTickStart() {
  TICK current = getCurrentTick();
  
  if( m_requestLog.end()!=m_nextRequest ) {
    if( current>m_nextRequest->first ) 
      syslog()<<"WARNING: missed some request posting. I post them now";
    for( ; m_requestLog.end()!=m_nextRequest && m_nextRequest->first<=current; ++m_nextRequest)
      postGoal(m_nextRequest->second);
  }
}

bool LogPlayer::synchronize() {
  TICK current = getCurrentTick();
  
  if( m_recallLog.end()!=m_nextRecall ) {
    if( current>m_nextRecall->first ) 
      syslog()<<"WARNING: missed some recall posting. I post them now";
    for( ; m_recallLog.end()!=m_nextRecall && m_nextRecall->first<=current; ++m_nextRecall)
      postRecall(m_nextRecall->second);
  }
  if( m_obsLog.end()==m_nextObs ) {
    syslog()<<"End of observation log => failed to synchronize.";
    return false;
  } else {
    if( current>m_nextObs->first ) {
      syslog()<<"ERROR: missed some observation posting.";
      return false;
    }
    for( ; m_obsLog.end()!=m_nextObs && m_nextObs->first==current; 
	 ++m_nextObs) 
      postObservation(m_nextObs->second);
    return true;
  }
}
