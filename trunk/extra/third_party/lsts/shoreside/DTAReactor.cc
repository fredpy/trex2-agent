#include "DTAReactor.hh"

#include <trex/domain/float_domain.hh>
#include <trex/domain/enum_domain.hh>
#include <trex/domain/boolean_domain.hh>

#include <boost/tokenizer.hpp>


using namespace TREX::LSTS;
using namespace TREX::transaction;
using namespace TREX::utils;


namespace
{
  /** @brief PositionUpdater reactor declaration */
  reactor::factory::declare<DTAReactor> decl("DTAReactor");
}


#define SURVEY_TL "lsts_survey" // internal



#define STATE_TL "lauv_state" // external -> auv
#define TREX_TL "drifterFollow" // exterba

/*
 * class DTAReactor
 */

// structors 

DTAReactor::DTAReactor(reactor::xml_arg_type arg)
  :reactor(arg, false),
   m_active(false),
m_proxy_timeline(parse_attr<symbol>(factory::node(arg), "proxy")),
m_asset_id(parse_attr<symbol>(factory::node(arg), "id")) {
  m_survey_tl = m_asset_id.str() + "_follow";
  m_state_tl = m_asset_id.str() + "_state";
  provide(m_survey_tl);
  post_observation(Observation(m_survey_tl, "None"));
  provide(m_state_tl, false);
  post_observation(Observation(m_state_tl, "Nothing"));
  use(m_proxy_timeline, true);
}

DTAReactor::~DTAReactor() {}

// callbacks 

bool DTAReactor::synchronize() {
  if( m_active ) {
    if( m_have_pos && WAITING==m_trex_state ) {
      Goal tmp(m_proxy_timeline, "Survey");
      Observation sent(m_state_tl, "Sent");
      
      
      tmp.restrictAttribute(var("center_lat", float_domain(m_pos.first)));
      sent.restrictAttribute(var("center_lat", float_domain(m_pos.first)));
      tmp.restrictAttribute(var("center_lon", float_domain(m_pos.second)));
      sent.restrictAttribute(var("center_lon", float_domain(m_pos.second)));
      
      enum_domain path_d;
      path_d.add(m_path);
      tmp.restrictAttribute(var("path", path_d));
    
      if( !m_have_speed ) {
        // set default speeds
        // In theory if they are not set the vehcile side know how to set them
        // although lets do most of the work where the cpu is
        m_speed.first = 0.0;
        m_speed.second = 0.0;
      }
      tmp.restrictAttribute(var("speed_north",
                                     float_domain(m_speed.first)));
      sent.restrictAttribute(var("speed_north",
                                     float_domain(m_speed.first)));
      tmp.restrictAttribute(var("speed_east",
                                     float_domain(m_speed.second)));
      sent.restrictAttribute(var("speed_east",
                                     float_domain(m_speed.second)));
    
      
      tmp.restrictAttribute(var("size", float_domain(m_factor)));
      tmp.restrictAttribute(var("lagrangian", boolean_domain(m_lagrangian)));
    
      post_goal(tmp);
      m_trex_state = GOAL_SENT;
      post_observation(sent);
    }
  }
  return true;
}

void DTAReactor::handle_request(goal_id const &g) {
  if( g->object()==m_survey_tl ) {
    if( g->predicate()=="None" ) {
      if( m_active ) {
        m_active=false;
        post_observation(Observation(m_survey_tl, g->predicate()));
        post_observation(Observation(m_state_tl, "Nothing"));
        unuse(m_drifter);
      }
    }
    else if( g->predicate()=="Track" ) {
      // Need a drifter, a size and a path
      TREX::utils::symbol drifter, path;
      double factor;

      if( g->hasAttribute("drifter") ) 
        drifter = g->getAttribute("drifter").domain().get_singleton_as_string();
      else 
        return;

      if( g->hasAttribute("path") ) 
        path = g->getAttribute("path").domain().get_singleton_as_string();
      else 
        return;
      if( g->hasAttribute("size") )
        factor = g->getAttribute("size").domain().get_typed_singleton<double, true>();
      else
        return;

      if( g->hasAttribute("lagrangian") )
        m_lagrangian = g->getAttribute("lagrangian").domain().get_typed_singleton<bool, true>();
      else 
        m_lagrangian = false;

      if( m_active ) {
        if( m_drifter!=drifter ) {
          unuse(m_drifter);
          m_drifter = drifter;
          use(m_drifter, false);
        }
      }
      else {
        m_drifter = drifter;
        use(m_drifter, false);
        m_active = true;
        m_have_pos = false;
      }
      m_path = path;
      m_factor = factor;
      post_observation(*g);
      post_observation(Observation(m_state_tl, "Wait"));
    }
  }
}

void DTAReactor::notify(Observation const &obs) {
  if( obs.object()==m_drifter ) {
    if( obs.predicate()=="position" || obs.predicate()=="connected" ) {
      // Get lat/lon
      m_pos.first = obs.getAttribute("latitude").domain().get_typed_singleton<double, true>();
      m_pos.second = obs.getAttribute("longitude").domain().get_typed_singleton<double, true>();
      if( !m_have_pos ) {
	syslog()<<"Received a first position from "<<m_drifter;
	m_have_pos = true;
      }
      if( obs.hasAttribute("speed_north") ) {
	m_have_speed = obs.getAttribute("speed_north").domain().is_singleton();
	m_speed.first = obs.getAttribute("speed_north").domain().get_typed_singleton<double, true>();
	m_speed.second = obs.getAttribute("speed_east").domain().get_typed_singleton<double, true>();
      }
    }
  } else if( obs.object()==m_proxy_timeline ) {
    if( obs.predicate()=="undefined" || obs.predicate()=="Failed" ) {
      syslog(warn)<<"No info from "<<m_proxy_timeline;
      m_trex_state = UNKNOWN;
    } else if( obs.predicate()=="Inactive" ) {
      syslog(info)<<"dorado waits for a new goal !!!";
      m_trex_state = WAITING;
    } else if( RUNNING!=m_trex_state ) {
      syslog(info)<<"dorado execute a goal !!!";
      m_trex_state = RUNNING;
      if( m_active )
      post_observation(Observation(m_state_tl, "Wait"));
    }
  }
}
