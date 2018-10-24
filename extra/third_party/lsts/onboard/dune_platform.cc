#include <DUNE/IMC/Definitions.hpp>
#include <DUNE/Network/Address.hpp>
#include <DUNE/Coordinates/WGS84.hpp>
#undef likely

#include "dune_platform.hh"

#include <trex/domain/FloatDomain.hh>

using namespace trex_lsts;
using namespace TREX::transaction;
using namespace TREX::utils;

namespace {
  
  TeleoReactor::xml_factory::declare<dune_platform> decl("Platform");
  
} // ::

/*
 * class trex_lsts::dune_platform
 */

// statics

Symbol const dune_platform::s_reference("reference");
Symbol const dune_platform::s_go("Going");
Symbol const dune_platform::s_at("At");
Symbol const dune_platform::s_lat("latitude");
Symbol const dune_platform::s_lon("longitude");


// structors

dune_platform::dune_platform(xml_arg_type arg):lsts_reactor(arg), m_first_tick(true), m_blocked(true), m_connected(false) {
  boost::property_tree::ptree::value_type &cfg = xml_factory::node(arg);
  
  m_env->reset_platform(this);
  
  bool is_uav = parse_attr<bool>(false, cfg, "uav");
  
  // DUNE sensors
  imc_provide<DUNE::IMC::EstimatedState>("estate", false, &dune_platform::on_estate, "Boot");
  imc_provide<DUNE::IMC::VehicleMedium>("medium", false, &dune_platform::on_medium, "Unknown");
  imc_provide<DUNE::IMC::PlanControl>("control", false, &dune_platform::on_control, "Boot");
  imc_provide<DUNE::IMC::OperationalLimits>("oplimits", false, &dune_platform::on_limits, "Boot");
  

  provide("refstate", false);
  
  if( is_uav ) {
    syslog(log::info)<<"Setting platform type to UAV (aerial)";
    m_go = boost::bind(&dune_platform::uav_go, this, _1);
    m_max_silent = 3;
  } else {
    syslog(log::info)<<"Setting platform type to AUV (underwater)";
    m_go = boost::bind(&dune_platform::auv_go, this, _1);
    m_max_silent = 1;
    
    // AUV specific sensors
    provide("ctd", false);     // ctd sensor data
    provide("ecopuck", false); // ecopuck data
  }
  
  // Timeline handling commands
  provide(s_reference, true);
  
  

  int trex_id = parse_attr<int>(cfg, "imcid");
  std::string dune_ip = parse_attr<std::string>("127.0.0.1", cfg, "duneip");
  int dune_port = parse_attr<int>(6002, cfg, "duneport");
  int local_port = parse_attr<int>(cfg, "localport");
  
  m_adapter->set_graph(get_graph());
  m_adapter->init_connection(local_port, trex_id, dune_ip, dune_port);
  
}

dune_platform::~dune_platform() {
  for(conn_map::iterator i=m_handlers.begin(); m_handlers.end()!=i; ++i)
    i->second.disconnect();
  m_env->reset_platform();
}

// trex handles

void dune_platform::handleInit() {
}

void dune_platform::handleTickStart() {
}

void dune_platform::handleRequest(goal_id const &g) {
  
}

void dune_platform::handleRecall(goal_id const &g) {
  
}

bool dune_platform::synchronize() {
  return true;
}

// manipulators


void dune_platform::on_estate(TREX::utils::Symbol const &tl, SHARED_PTR<DUNE::IMC::EstimatedState> state) {
  if( state ) {
    Observation o(tl, "Position");
    FloatDomain::base_type latitude, longitude;
    
    latitude = state->lat;
    longitude = state->lon;
    DUNE::Coordinates::WGS84::displace(state->x, state->y, &latitude, &longitude);
    
    o.restrictAttribute(s_lat, FloatDomain(latitude));
    o.restrictAttribute(s_lon, FloatDomain(longitude));
    o.restrictAttribute("psi", FloatDomain(state->psi));
    
    if( 0 < state->depth ) {
      o.restrictAttribute("z", FloatDomain(state->depth));
      o.restrictAttribute("depth", FloatDomain(state->depth));
    } else if( 0 < state->alt )
      o.restrictAttribute("z", FloatDomain(state->alt));
    else if( -1 != state->height )
      o.restrictAttribute("z", FloatDomain(state->height));
    
    if( 0 < state->alt )
      o.restrictAttribute("altitude", FloatDomain(state->alt));
    if( -1 != state->height )
      o.restrictAttribute("height", FloatDomain(state->height));
    
    postObservation(o, false);
  }
}

void dune_platform::on_medium(TREX::utils::Symbol const &tl, SHARED_PTR<DUNE::IMC::VehicleMedium> medium) {
  
}

void dune_platform::on_control(TREX::utils::Symbol const &tl, SHARED_PTR<DUNE::IMC::PlanControl> control) {
  
}

void dune_platform::on_limits(TREX::utils::Symbol const &tl, SHARED_PTR<DUNE::IMC::OperationalLimits> limits) {
  
}



bool dune_platform::auv_go(TREX::transaction::goal_id g) {
  return false;
}

bool dune_platform::uav_go(TREX::transaction::goal_id g) {
  return false;
}


bool dune_platform::send(imc_adapter::message &m) {
  m.setTimeStamp();
  return m_adapter->send(m);
}

