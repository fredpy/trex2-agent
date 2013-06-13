#include "DTAReactor.hh"
#include "../shared/earth_point.hh"

#include <trex/domain/FloatDomain.hh>
#include <trex/domain/EnumDomain.hh>
#include <trex/domain/BooleanDomain.hh>

#include <boost/tokenizer.hpp>


using namespace mbari::iridium;
using namespace TREX::transaction;
using namespace TREX::utils;

namespace tlog=TREX::utils::log;

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
  m_current_end.reset(new IntegerDomain());

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
  update_state(getCurrentTick());
  
  if( m_active )
    if( m_have_pos && WAITING==m_trex_state )
      send_cmd();
  return true;
}

void DTAReactor::send_cmd() {
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
  report.restrictAttribute("latitude", FloatDomain(pos.latitude()));
  report.restrictAttribute("longitude", FloatDomain(pos.longitude()));
  report.restrictAttribute("lagrangian", BooleanDomain(m_lagrangian));
  report.restrictAttribute("path", EnumDomain(m_path));
  report.restrictAttribute("drifter", EnumDomain(m_drifter));
  postObservation(report);
}

void DTAReactor::update_state(TICK date) {
  // Check if I can change the state
  IntegerDomain future(date+1, IntegerDomain::plus_inf);
  
  
  if( m_current_end->contains(date) ) {
    bool state_changed = false;
    std::list<goal_id>::iterator i = m_pending_goals.begin();
    
    while( m_pending_goals.end()!=i ) {
      if( (*i)->startsAfter(date) ) {
        if( !state_changed ) {
          if( (*i)->startsBefore(date+1) ) {
            // I need a new goal and this one fits the bill
            (*i)->restrictStart(IntegerDomain(date));
            set_state(**i);
            state_changed = true;
          }
        }
      } else {
        syslog(tlog::warn)<<"Rejected goal "<<*i;
        i = m_pending_goals.erase(i);
      }
    }
 
    
    if( !state_changed ) {
      if( m_current_end->contains(date+1) ) {
        m_current_end->restrictWith(future);
      } else {
        syslog(tlog::info)<<"Completed last goal; rolling back to None";
        set_state(Observation(SURVEY_TL, "None"), future);
      }
    }
  }
}

void DTAReactor::set_state(Observation obs, IntegerDomain const &end) {
  if( obs.predicate()=="None" ) {
    if( m_active ) {
      m_active = false;
      postObservation(Observation(STATE_TL, "Nothing"));
      unuse(tl_name(m_drifter));
      postObservation(Observation(TRACK_TL, "None"));
    }
  } else if( obs.predicate()=="Track" ) {
    Symbol drifter;

    m_lagrangian = false;
    
    drifter = obs.getAttribute("drifter").domain().getStringSingleton();
    m_path = obs.getAttribute("path").domain().getStringSingleton();
    m_factor = obs.getAttribute("size").domain().getTypedSingleton<double, true>();
    
    if( obs.hasAttribute("lagrangian") )
      m_lagrangian = obs.getAttribute("lagrangian").domain().getTypedSingleton<bool, true>();
    else
      obs.restrictAttribute("lagrangian", BooleanDomain(false));
    
    if( obs.hasAttribute("shift_north") )
      m_shift.first = obs.getAttribute("shift_north").domain().getTypedSingleton<double, true>();
    else {
      m_shift.first = 0.0;
      obs.restrictAttribute("shift_north", FloatDomain(0.0));
    }
    
    if( obs.hasAttribute("shift_east") )
      m_shift.second = obs.getAttribute("shift_east").domain().getTypedSingleton<double, true>();
    else {
      m_shift.second = 0.0;
      obs.restrictAttribute("shift_east", FloatDomain(0.0));
    }
    
    if( m_active ) {
      if( m_drifter!=drifter ) {
        unuse(tl_name(m_drifter));
        m_active = false;
      }
    }
    if( !m_active ) {
      m_drifter = drifter;
      m_active = true;
      Observation track_o(TRACK_TL, "NoPosition");
      track_o.restrictAttribute("drifter", EnumDomain(m_drifter));
      postObservation(track_o);
      use(tl_name(m_drifter));
    }
    postObservation(Observation(STATE_TL, "Wait"));
  }
  postObservation(obs);
  m_current_end.reset(new IntegerDomain(end));
}



void DTAReactor::handleRequest(goal_id const &g) {
  if( g->object()==SURVEY_TL ) {
    // basic checks
    if( g->predicate()=="None" )
      m_pending_goals.push_back(g);
    else if( g->predicate()=="Track" ) {
      if( !( g->hasAttribute("drifter") &&
            g->getAttribute("drifter").domain().isSingleton() ) ) {
        syslog(tlog::warn)<<"Ignoring track goal ["<<g<<"] : missing drifter";
        return;
      }
      if( !( g->hasAttribute("path") &&
            g->getAttribute("path").domain().isSingleton() ) ) {
        syslog(tlog::warn)<<"Ignoring track goal ["<<g<<"] : missing path";
        return;
      }
      if( !( g->hasAttribute("size") &&
            g->getAttribute("size").domain().isSingleton() ) ) {
        syslog(tlog::warn)<<"Ignoring track goal ["<<g<<"] : missing size";
        return;
      }
      m_pending_goals.push_back(g);
    } else
      syslog(tlog::warn)<<"Ignoring goal ["<<g<<"] with unknown predicate "<<g->predicate();
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
