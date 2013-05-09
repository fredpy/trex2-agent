#include "DTAReactor.hh"

#include <trex/domain/FloatDomain.hh>
#include <trex/domain/EnumDomain.hh>
#include <trex/domain/BooleanDomain.hh>

#include <boost/tokenizer.hpp>


using namespace TREX::LSTS;
using namespace TREX::transaction;
using namespace TREX::utils;


namespace
{
  /** @brief PositionUpdater reactor declaration */
  TeleoReactor::xml_factory::declare<DTAReactor> decl("DTAReactor");
}


#define SURVEY_TL "lsts_survey" // internal



#define STATE_TL "lauv_state" // external -> auv
#define TREX_TL "drifterFollow" // exterba

/*
 * class DTAReactor
 */

// structors 

DTAReactor::DTAReactor(TeleoReactor::xml_arg_type arg) 
  :TeleoReactor(arg, false), 
   m_active(false),
m_proxy_timeline(parse_attr<Symbol>(xml_factory::node(arg), "proxy")),
m_asset_id(parse_attr<Symbol>(xml_factory::node(arg), "id")) {
  m_survey_tl = m_asset_id.str() + "_follow";
  m_state_tl = m_asset_id.str() + "_state";
  provide(m_survey_tl);
  postObservation(Observation(m_survey_tl, "None"));
  provide(m_state_tl, false);
  postObservation(Observation(m_state_tl, "Nothing"));
  use(m_proxy_timeline, true);
}

DTAReactor::~DTAReactor() {}

// callbacks 

bool DTAReactor::synchronize() {
  if( m_active ) {
    if( m_have_pos && WAITING==m_trex_state ) {
      Goal tmp(m_proxy_timeline, "Survey");
      tmp.restrictAttribute(Variable("center_lat", FloatDomain(m_pos.first)));
      tmp.restrictAttribute(Variable("center_lon", FloatDomain(m_pos.second)));
      
      EnumDomain path_d;
      path_d.add(m_path);
      tmp.restrictAttribute(Variable("path", path_d));
    
      if( !m_have_speed ) {
        // set default speeds
        // In theory if they are not set the vehcile side know how to set them
        // although lets do most of the work where the cpu is
        m_speed.first = 0.0;
        m_speed.second = 0.0;
      }
      tmp.restrictAttribute(Variable("speed_north",
                                     FloatDomain(m_speed.first)));
      tmp.restrictAttribute(Variable("speed_east",
                                     FloatDomain(m_speed.second)));
    
      
      tmp.restrictAttribute(Variable("size", FloatDomain(m_factor)));
      tmp.restrictAttribute(Variable("lagrangian", BooleanDomain(m_lagrangian)));
    
      postGoal(tmp);
    }
  }
  return true;
}

void DTAReactor::handleRequest(goal_id const &g) {
  if( g->object()==m_survey_tl ) {
    if( g->predicate()=="None" ) {
      if( m_active ) {
	m_active=false;
	postObservation(Observation(m_survey_tl, g->predicate()));
	postObservation(Observation(m_state_tl, "Nothing"));
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

      if( g->hasAttribute("lagrangian") )
        m_lagrangian = g->getAttribute("lagrangian").domain().getTypedSingleton<bool, true>();
      else 
        m_lagrangian = false;

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
      postObservation(Observation(m_state_tl, "Wait"));
    }
  }
}

void DTAReactor::notify(Observation const &obs) {
  if( obs.object()==m_drifter ) {
    if( obs.predicate()=="position" || obs.predicate()=="connected" ) {
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
  } else if( obs.object()==m_proxy_timeline ) {
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
      postObservation(Observation(m_state_tl, "Wait"));
    }
  }
}
