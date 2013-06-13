#include "DTAReactor.hh"
#include "../shared/earth_point.hh"

#include <trex/domain/FloatDomain.hh>
#include <trex/domain/EnumDomain.hh>

#include <boost/tokenizer.hpp>


using namespace mbari::iridium;
using namespace TREX::transaction;
using namespace TREX::utils;

#define SURVEY_TL "dorado_survey"
#define STATE_TL  "dorado_state"
#define TRACK_TL  "dorado_tracked"
#define TREX_TL "drifterFollow"
#define IRIDIUM_TL "iridium"

/*
 * class DTAReactor
 */

// structors 

DTAReactor::DTAReactor(TeleoReactor::xml_arg_type arg) 
  :TeleoReactor(arg, false), 
   m_active(false),
   m_imei(parse_attr<std::string>(TeleoReactor::xml_factory::node(arg), "imei")),
   m_iridium(parse_attr<std::string>(TeleoReactor::xml_factory::node(arg), "host")),
   m_drifter_pfx(parse_attr<std::string>("_pos_", TeleoReactor::xml_factory::node(arg), "prefix")) {
  provide(SURVEY_TL);
  postObservation(Observation(SURVEY_TL, "None"));
  provide(STATE_TL, false);
  postObservation(Observation(STATE_TL, "Nothing"));
     
  provide(TRACK_TL, false);
  postObservation(Observation(TRACK_TL, "None"));

     
  use(TREX_TL, false);
  m_iridium.set_sender(parse_attr<std::string>(TeleoReactor::xml_factory::node(arg), "from"));
  syslog(info)<<"Mail sender set to "<<m_iridium.sender();

  m_use_iridium = parse_attr<bool>(true, TeleoReactor::xml_factory::node(arg), 
				      "iridium");
     
  if( m_use_iridium ) {
    m_iridium.add_recipient(SbdMailer::s_iridium_address);
    syslog(info)<<"Adding iridium to the recipients";
  } else {
    syslog(info)<<"No iridium: providing timeline to activate it";
    provide(IRIDIUM_TL);
    postObservation(Observation(IRIDIUM_TL, "Inactive"));
  }
    
  std::string to = parse_attr<std::string>("", TeleoReactor::xml_factory::node(arg),
					   "to");
  boost::char_delimiters_separator<char> sep(false, "", ",");
  boost::tokenizer<> tok(to, sep);
  for(boost::tokenizer<>::iterator i=tok.begin(); tok.end()!=i; ++i) {
    if( !i->empty() ) {
      m_iridium.add_recipient(*i);
      syslog(info)<<"Adding "<<*i<<" to the recipients";
    }
  }
//   syslog(info)<<"SMTP connection to "<<parse_attr<std::string>(TeleoReactor::xml_factory::node(arg), "host"); 
//   m_iridium.login();
//   m_iridium.close();
//   syslog(info)<<"SMTP connection succeeded."; 
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
      trex_goal.precision(8);
      mbari::earth_point drifter(m_pos.first, m_pos.second), pos;
      
      mbari::rhumb_lines calc;
      pos = drifter.destination(m_shift.first, m_shift.second, calc);
      
      
      trex_goal<<"<Goal on=\""<<TREX_TL<<"\" predicate=\"DrifterFollow.Survey\">\n"
	       <<"  <Assert name=\"centerLat\"><value type=\"LATITUDE\" name=\""<<pos.latitude()<<"\" /></Assert>\n"
	       <<"  <Assert name=\"centerLon\"><value type=\"LONGITUDE\" name=\""<<pos.longitude()<<"\" /></Assert>\n"
	       <<"  <Assert name=\"path\"><object value=\""<<m_path<<"\" /></Assert>\n"
	       <<"  <Assert name=\"size\"><value type=\"float\" name=\""<<m_factor<<"\" /></Assert>\n"
	       <<"  <Assert name=\"lagrangian\"><value type=\"bool\" name=\""<<(m_lagrangian?"true":"false")<<"\" /></Assert>\n"
	       <<"  <Assert name=\"speedNorth\"><value type=\"float\" name=\""<<m_speed.first<<"\" /></Assert>\n"
	       <<"  <Assert name=\"speedEast\"><value type=\"float\" name=\""<<m_speed.second<<"\" /></Assert>\n"
	       <<"</Goal>";
      info.precision(8);
      if( m_lagrangian )
        info<<"Lagrangian ";
      else
        info<<"world frame ";
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
	unuse(tl_name(m_drifter));
        postObservation(Observation(TRACK_TL, "None"));
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

      if( g->hasAttribute("lagrangian") )
        m_lagrangian = g->getAttribute("lagrangian").domain().getTypedSingleton<bool, true>();
      else 
        m_lagrangian = false;
      
      if( g->hasAttribute("shift_north") )
        m_shift.first = g->getAttribute("shift_north").domain().getTypedSingleton<double, true>();
      else {
        m_shift.first = 0.0;
        g->restrictAttribute(Variable("shift_north", FloatDomain(0.0)));
      }

      if( g->hasAttribute("shift_east") )
        m_shift.second = g->getAttribute("shift_east").domain().getTypedSingleton<double, true>();
      else {
        m_shift.second = 0.0;
        g->restrictAttribute(Variable("shift_east", FloatDomain(0.0)));
      }

      if( m_active ) {
	if( m_drifter!=drifter ) {
	  unuse(tl_name(m_drifter));
	  m_drifter = drifter;
	  use(tl_name(m_drifter), false);
	}
     } else {
	m_drifter = drifter;
	use(tl_name(m_drifter), false);
        Observation obs(TRACK_TL, "NoPosition");
        obs.restrictAttribute("drifter", EnumDomain(m_drifter));
        postObservation(obs);
	m_active = true;
	m_have_pos = false;
      }
      m_path = path;
      m_factor = factor;
      postObservation(*g);
      postObservation(Observation(STATE_TL, "Wait"));
    }
  } else if( g->object()==IRIDIUM_TL ) {
    if( g->predicate()=="Ping" ) {
      std::ostringstream msg;
      msg<<*g;
      std::string str = msg.str();
      m_iridium.send(m_imei, str.c_str(), str.length(), "Ping");
    } if( !m_use_iridium ) {
      if( g->predicate()=="Active" ) {
	m_use_iridium =true;
	m_iridium.add_recipient(SbdMailer::s_iridium_address);
	syslog(info)<<"Adding iridium to the recipients";
	postObservation(*g);
      }
    }
  }
}

void DTAReactor::notify(Observation const &obs) {
  if( obs.object()==tl_name(m_drifter) ) {
    if( obs.predicate()=="Holds" ) {      
      // Get lat/lon
      m_pos.first = obs.getAttribute("latitude").domain().getTypedSingleton<double, true>();
      m_pos.second = obs.getAttribute("longitude").domain().getTypedSingleton<double, true>();

      Observation update(TRACK_TL, "Position");
      update.restrictAttribute("drifter", EnumDomain(m_drifter));
      update.restrictAttribute(obs.getAttribute("latitude"));
      update.restrictAttribute(obs.getAttribute("longitude"));
      if( !m_have_pos ) {
	syslog()<<"Received a first position from "<<m_drifter;
	m_have_pos = true;
      }
      if( obs.hasAttribute("speed_north") ) {
	m_have_speed = obs.getAttribute("speed_north").domain().isSingleton();
	m_speed.first = obs.getAttribute("speed_north").domain().getTypedSingleton<double, true>();
        update.restrictAttribute(obs.getAttribute("speed_north"));
	m_speed.second = obs.getAttribute("speed_east").domain().getTypedSingleton<double, true>();
        update.restrictAttribute(obs.getAttribute("speed_east"));
      }
      postObservation(update);
    }
  } else if( obs.object()==TREX_TL ) {
    if( obs.predicate()=="undefined" || obs.predicate()=="Failed" ) {
      syslog(warn)<<"No info from dorado";
      m_trex_state = UNKNOWN;
    } else if( obs.predicate()=="Inactive" ) {
      syslog(info)<<"dorado waits for a new goal !!!";
      m_trex_state = WAITING;
    } else if( RUNNING!=m_trex_state ) {
      syslog(info)<<"dorado execute a goal !!!";
      m_trex_state = RUNNING;
      if( m_active )
      postObservation(Observation(STATE_TL, "Wait"));
    }
  }
}
