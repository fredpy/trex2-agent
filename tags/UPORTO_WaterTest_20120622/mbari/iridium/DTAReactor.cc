#include "DTAReactor.hh"

#include <trex/domain/FloatDomain.hh>
#include <trex/domain/EnumDomain.hh>

#include <boost/tokenizer.hpp>


using namespace mbari::iridium;
using namespace TREX::transaction;
using namespace TREX::utils;

#define SURVEY_TL "dorado_survey"
#define STATE_TL "dorado_state"
#define TREX_TL "drifterFollow"

/*
 * class DTAReactor
 */

// structors 

DTAReactor::DTAReactor(TeleoReactor::xml_arg_type arg) 
  :TeleoReactor(arg, false), 
   m_active(false),
   m_imei(parse_attr<std::string>(TeleoReactor::xml_factory::node(arg), "imei")),
   m_iridium(parse_attr<std::string>(TeleoReactor::xml_factory::node(arg), "host")) {
  provide(SURVEY_TL);
  postObservation(Observation(SURVEY_TL, "None"));
  provide(STATE_TL, false);
  postObservation(Observation(STATE_TL, "Nothing"));
  use(TREX_TL, false);
  m_iridium.set_sender(parse_attr<std::string>(TeleoReactor::xml_factory::node(arg), "from"));
  syslog("INFO")<<"Mail sender set to "<<m_iridium.sender();

  bool use_iridium = parse_attr<bool>(true, TeleoReactor::xml_factory::node(arg), 
				      "iridium");
  if( use_iridium ) {
    m_iridium.add_recipient(SbdMailer::s_iridium_address);
    syslog("INFO")<<"Adding iridium to the recipients";
  }
  std::string to = parse_attr<std::string>("", TeleoReactor::xml_factory::node(arg),
					   "to");
  boost::char_delimiters_separator<char> sep(false, "", ",");
  boost::tokenizer<> tok(to, sep);
  for(boost::tokenizer<>::iterator i=tok.begin(); tok.end()!=i; ++i) {
    if( !i->empty() ) {
      m_iridium.add_recipient(*i);
      syslog("INFO")<<"Adding "<<*i<<" to the recipients";
    }
  }
  syslog("INFO")<<"SMTP connection to "<<parse_attr<std::string>(TeleoReactor::xml_factory::node(arg), "host"); 
  m_iridium.login();
  syslog("INFO")<<"SMTP connection succeeded."; 
}

DTAReactor::~DTAReactor() {}

// callbacks 

bool DTAReactor::synchronize() {
  if( m_active ) {
    if( m_have_pos && WAITING==m_trex_state ) {
      std::ostringstream trex_goal, info;
      if( !m_have_speed ) {
	m_speed.first = 0.0;
	m_speed.second = 0.0;
      }
      trex_goal<<"<Goal on=\""<<TREX_TL<<"\" predicate=\"DrifterFollow.Survey\">\n"
	       <<"  <Assert name=\"centerLat\"><value type=\"LATITUDE\" name=\""<<m_pos.first<<"\" /></Assert>\n"
	       <<"  <Assert name=\"centerLon\"><value type=\"LONGITUDE\" name=\""<<m_pos.second<<"\" /></Assert>\n"
	       <<"  <Assert name=\"path\"><object value=\""<<m_path<<"\" /></Assert>\n"
	       <<"  <Assert name=\"size\"><value type=\"float\" name=\""<<m_factor<<"\" /></Assert>\n"
	       <<"  <Assert name=\"lagrangian\"><value type=\"bool\" name=\"false\" /></Assert>\n"
	       <<"  <Assert name=\"speedNorth\"><value type=\"float\" name=\""<<m_speed.first<<"\" /></Assert>\n"
	       <<"  <Assert name=\"speedEast\"><value type=\"float\" name=\""<<m_speed.second<<"\" /></Assert>\n"
	       <<"</Goal>";
      info<<m_path<<" of "<<m_factor<<"m around ("<<m_pos.first<<", "<<m_pos.second
	  <<")\n\t direction : "<<m_speed.first<<"N, "<<m_speed.second<<"E\n\n";
      syslog()<<"Sending new SBD goal:\n\t"<<info.str();

      std::string msg = trex_goal.str();
      m_iridium.send(m_imei, msg.c_str(), msg.length(), info.str());
      m_trex_state = GOAL_SENT;
      Observation report(STATE_TL, "Sent");
      report.restrictAttribute("latitude", FloatDomain(m_pos.first));
      report.restrictAttribute("longitude", FloatDomain(m_pos.second));
      report.restrictAttribute("path", EnumDomain(m_path));
      report.restrictAttribute("drifter", EnumDomain(m_drifter));
      postObservation(report);
    }
  }
  return true;
}

void DTAReactor::handleRequest(goal_id const &g) {
  if( g->object()==SURVEY_TL ) {
    if( g->predicate()=="None" ) {
      if( m_active ) {
	m_active=false;
	postObservation(Observation(SURVEY_TL, g->predicate()));
	postObservation(Observation(STATE_TL, "Nothing"));
	unuse(m_drifter);
      }
    } else if( g->predicate()=="Track" ) {
      // Need a drifter, a size and a path
      TREX::utils::Symbol drifter, path;
      double factor;

      if( g->hasAttribute("drifter") ) 
	drifter = g->getAttribute("drifter").domain().getStringSingleton();
      else 
	return;

      if( g->hasAttribute("path") ) 
	path = g->getAttribute("path").domain().getStringSingleton();
      else 
	return;
      if( g->hasAttribute("size") )
	factor = g->getAttribute("size").domain().getTypedSingleton<double, true>();
      else
	return;
      // Will need to add lagrangian flag ... 
      if( m_active ) {
	if( m_drifter!=drifter ) {
	  unuse(m_drifter);
	  m_drifter = drifter;
	  use(m_drifter, false);
	}
      } else {
	m_drifter = drifter;
	use(m_drifter, false);
	m_active = true;
	m_have_pos = false;
      }
      m_path = path;
      m_factor = factor;
      postObservation(*g);
      postObservation(Observation(STATE_TL, "Wait"));
    }
  }
}

void DTAReactor::notify(Observation const &obs) {
  if( obs.object()==m_drifter ) {
    if( obs.predicate()=="Holds" ) {
      // Get lat/lon
      m_pos.first = obs.getAttribute("latitude").domain().getTypedSingleton<double, true>();
      m_pos.second = obs.getAttribute("longitude").domain().getTypedSingleton<double, true>();
      if( !m_have_pos ) {
	syslog()<<"Received a first position from "<<m_drifter;
	m_have_pos = true;
      }
      if( obs.hasAttribute("speed_north") ) {
	m_have_speed = obs.getAttribute("speed_north").domain().isSingleton();
	m_speed.first = obs.getAttribute("speed_north").domain().getTypedSingleton<double, true>();
	m_speed.second = obs.getAttribute("speed_east").domain().getTypedSingleton<double, true>();
      }
    }
  } else if( obs.object()==TREX_TL ) {
    if( obs.predicate()=="undefined" || obs.predicate()=="Failed" ) {
      syslog("WARN")<<"No info from dorado";
      m_trex_state = UNKNOWN;
    } else if( obs.predicate()=="Inactive" ) {
      syslog("INFO")<<"dorado waits for a new goal !!!";
      m_trex_state = WAITING;
    } else if( RUNNING!=m_trex_state ) {
      syslog("INFO")<<"dorado execute a goal !!!";
      m_trex_state = RUNNING;
      if( m_active )
      postObservation(Observation(STATE_TL, "Wait"));
    }
  }
}