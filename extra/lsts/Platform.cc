/** @file "Platform.cc"
 * @brief Platform plugin implementation
 *
 * @author Jose Pinto <zepinto@gmail.com>
 * @ingroup lsts
 */
#include <iostream>

#include "EuropaExtensions.hh"

#include <trex/utils/Plugin.hh>
#include <trex/utils/LogManager.hh>
#include <trex/domain/IntegerDomain.hh>
#include <trex/domain/FloatDomain.hh>
#include <trex/domain/StringDomain.hh>
#include <trex/domain/BooleanDomain.hh>

#include <Dune/Math/Angles.hpp>
#include <Dune/Coordinates/WGS84.hpp>
#include "Platform.hh"

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::LSTS;


namespace
{

/** @brief TREX log entry point */
SingletonUse<LogManager> s_log;

/** @brief Platform reactor declaration */
TeleoReactor::xml_factory::declare<Platform> decl("Platform");

}

namespace TREX
{

/** @brief Plug-in initialization
 *
 * This function is called by TREX after loading the plug-in.
 * It manage the initialization of this plug-in
 *
 * @ingroup lsts
 */
void
initPlugin()
{
	::s_log->syslog("plugin.platform") << "Platform loaded." << std::endl;
}

} // TREX

std::map<uint16_t, IMC::Message *> received;
std::map<uint16_t, IMC::Message *> aggregate;
IMC::VehicleCommand lastCommand;
bool estate_posted = false, oplimits_posted = false, was_idle = false, supervision_posted = false;
int last_vstate = -1;
int remote_id = 0;
bool supervision_blocked = false;

ControlInterface * Platform::controlInterfaceInstance = 0;


Platform::Platform(TeleoReactor::xml_arg_type arg) :
                        						TeleoReactor(arg, false), m_on(
                        								parse_attr<bool>(false, TeleoReactor::xml_factory::node(arg), "state")), m_firstTick(
                        										true)
{

	m_env->setPlatformReactor(this);

	duneport = parse_attr<int>(6002, TeleoReactor::xml_factory::node(arg),
			"duneport");

	duneip = parse_attr<std::string>("127.0.0.1", TeleoReactor::xml_factory::node(arg),
			"duneip");

	debug = parse_attr<bool>(false, TeleoReactor::xml_factory::node(arg), "debug");

	syslog("INFO") << "Connecting to dune on " << duneip << ":" << duneport;

	// start listening for dune in a new thread
	localport = parse_attr<int>(false, TeleoReactor::xml_factory::node(arg),
			"localport");

	bfr = new uint8_t[65535];
	receive.bind(localport, Address::Any, true);
	receive.addToPoll(iom);

	syslog("INFO") << "listening on port " << localport << "...";

	provide("estate", false); 		// declare the state command timeline
	provide("vstate", false); 		// declare the state command timeline
	provide("command"); 			// declare the command timeline
	provide("oplimits", false); 	// declare the oplimits timeline
	provide("maneuver", false);
	provide("supervision", false); 		// declare the trex command timeline

	IMC::CacheControl m;
	m.op = IMC::CacheControl::COP_LOAD;
	sendMsg(m);

	sleep(3);

}

Platform::~Platform()
{
	delete[] bfr;
}
void
Platform::setControlInterface(ControlInterface * itf)
{
	controlInterfaceInstance = itf;
}

bool
Platform::synchronize()
{
	try
	{
		Address addr;
		int msg_count = 0;
		//bool estate_changed = false, vstate_changed = false, oplimits_changed =
		// 		false, gpsfix_changed = false;

		//entityStates.clear();
		received.clear();

		while (iom.poll(0))
		{
			msg_count++;
			uint16_t rv = receive.read((char*)bfr, 65535, &addr);
			IMC::Message * msg = IMC::Packet::deserialize(bfr, rv);
			if (remote_id == 0)
				remote_id = msg->getSource();

			if (received.count(msg->getId()))
				delete received[msg->getId()];
			received[msg->getId()] = msg;

			if (msg->getId() == IMC::VehicleCommand::getIdStatic())
			{
				IMC::VehicleCommand * cmd = dynamic_cast<IMC::VehicleCommand *>(msg);
				if (cmd->type == IMC::VehicleCommand::VC_REQUEST)
				{
					lastCommand = *cmd;
					//if (!cmd->maneuver.isNull())
					//	cmd->maneuver->toText(std::cout);
				}
			}

			if (msg->getId() == IMC::TrexCommand::getIdStatic())
			{
				IMC::TrexCommand * command = dynamic_cast<IMC::TrexCommand*>(msg);

				switch (command->command)
				{
				case IMC::TrexCommand::OP_POST_GOAL:
					syslog("INFO") << "received a new goal (" << command->goalid << "): " << command->goalxml;
					if (controlInterfaceInstance) {
						controlInterfaceInstance->proccess_message(command->goalxml);
					}
					break;
				case IMC::TrexCommand::OP_ENABLE:
					syslog("WARN") << "Enable TREX command received";
					// post active observation ...
					postObservation(Observation("supervision", "Active"));
					reportToDune("Activate TREX command received");

					break;
				case IMC::TrexCommand::OP_DISABLE:
					syslog("WARN") << "Disable TREX command received";
					// post blocked observation ...
					postObservation(Observation("supervision", "Blocked"));
					reportToDune("Disable TREX command received");
					break;
				}
			}

			if (msg->getId() == IMC::Abort::getIdStatic())
			{
				syslog("WARN") << "Abort received";
				// post blocked observation ...
				postObservation(Observation("supervision", "Blocked"));
				reportToDune("Disabling TREX due to abort detection");
			}
		}

		std::map<uint16_t, IMC::Message *>::iterator it;

		for (it = received.begin(); it != received.end(); it++)
			aggregate[it->first] = it->second;

		processState();

		if (!supervision_posted) {
			postObservation(Observation("supervision", "Active"));
			supervision_posted = true;
		}

		// send an heartbeat to Dune
		IMC::Heartbeat hb;
		sendMsg(hb);

		syslog("INFO") << "processed " << msg_count << " messages\n";
		std::cout << "processed " << msg_count << " messages\n";
	}
	catch (std::runtime_error& e)
	{
		std::cerr << e.what();
		return false;
	}

	return true;
}


std::map<goal_id, IMC::Message *> future_goals;
goal_id current_maneuver;

void
Platform::handleRequest(goal_id const &g)
{
	Goal * goal = g.get();
	std::string gname = (goal->object()).str();
	std::string gpred = (goal->predicate()).str();
	std::string man_name;
	syslog("INFO") << "handleRequest(" << gname << "." << gpred << ")\n";

	std::ostringstream ss;
	ss << "TREX_goal_" << g;
	man_name = ss.str();

	if (gname.compare("command") == 0 && gpred.compare("Idle") == 0) {

		future_goals[g] = commandIdle(man_name);
		return;
	}
	if (gname.compare("command") == 0 && gpred.compare("Maneuver") == 0)
	{

		Variable latitude = goal->getAttribute("latitude");
		Variable longitude = goal->getAttribute("longitude");
		Variable depth = goal->getAttribute("depth");
		Variable secs = goal->getAttribute("secs");

		if (latitude.domain().isSingleton() && longitude.domain().isSingleton()
				&& depth.domain().isSingleton()) {
			double lat = latitude.domain().getTypedSingleton<double, false>();// latitude.domain().getSingleton());// atof(northing.domain().getStringSingleton().c_str());
			double lon = longitude.domain().getTypedSingleton<double, false>();

			double dep_v = depth.domain().getTypedSingleton<double, false>();//atof(depth.domain().getStringSingleton().c_str());

			if (secs.domain().isSingleton()
					&& secs.domain().getStringSingleton().compare("0") == 0)
				future_goals[g] = commandGoto(man_name, lat, lon, dep_v, 1.5);
			else if (secs.domain().isSingleton())
			{
				double dur = secs.domain().getTypedSingleton<double, false>();//atof(secs.domain().getStringSingleton().c_str());
				if (dep_v > 0)
					future_goals[g] = commandLoiter(man_name, lat, lon, dep_v, 15.0, 1.5, (int)dur);
				else
					future_goals[g] = commandStationKeeping(man_name, lat, lon, 1.5, (int)dur);
			}
		}
		else {
			syslog("ERROR") << "variables are not singletons!\n";
		}
	}
}

void
Platform::handleRecall(goal_id const &g)
{
	syslog("INFO") << "handleRecall(" << g << ")";
}

void
Platform::convertToRelative(double lat, double lon, double &x, double &y)
{
	if (!aggregate.count(IMC::HomeRef::getIdStatic()))
		return;

	IMC::HomeRef * homeRef =
			dynamic_cast<IMC::HomeRef *>(aggregate[IMC::HomeRef::getIdStatic()]);
	WGS84::displacement(homeRef->lat, homeRef->lon, 0, lat, lon, 0, &x, &y);
}

void
Platform::convertToAbsolute(double northing, double easting, double &lat, double &lon)
{
	// double tmp = 0;
	if (lat == 0 && lon == 0) {
		if (!aggregate.count(IMC::HomeRef::getIdStatic()))
			return;

		IMC::HomeRef * homeRef = dynamic_cast<IMC::HomeRef *>(aggregate[IMC::HomeRef::getIdStatic()]);

		lat = homeRef->lat;
		lon = homeRef->lon;
	}

	WGS84::displace(northing, easting, &lat, &lon);
}

void
Platform::processState()
{

	/* ESTIMATED_STATE */

	if (received.count(IMC::EstimatedState::getIdStatic()))
	{
		IMC::EstimatedState * msg =
				dynamic_cast<IMC::EstimatedState *>(aggregate[IMC::EstimatedState::getIdStatic()]);

		Observation obs("estate", "Position");
		double lat = msg->lat, lon = msg->lon;//, hae = 0.0;//, northing = msg->x, easting = msg->y;
		if (msg->ref == IMC::EstimatedState::RM_NED_LLD)
			WGS84::displace(msg->x, msg->y, &lat, &lon);

		obs.restrictAttribute("latitude", FloatDomain(lat));
		obs.restrictAttribute("longitude", FloatDomain(lon));
		obs.restrictAttribute("depth", FloatDomain(msg->z));

		if (aggregate.count(IMC::NavigationUncertainty::getIdStatic()))
		{
			IMC::NavigationUncertainty * navUnc =
					dynamic_cast<IMC::NavigationUncertainty *>(aggregate[IMC::NavigationUncertainty::getIdStatic()]);
			obs.restrictAttribute("uncertainty",
					FloatDomain(std::max(navUnc->x, navUnc->y)));
		}
		estate_posted = true;
		postObservation(obs);
	}
	else if (!estate_posted)
	{
		postObservation(
				Observation("estate", "undefined"));
	}

	/* OPERATIONAL_LIMITS */

	if (received.count(IMC::OperationalLimits::getIdStatic()))
	{
		IMC::OperationalLimits * msg =
				dynamic_cast<IMC::OperationalLimits *>(received[IMC::OperationalLimits::getIdStatic()]);
		Observation obs("oplimits", "Limits");

		if (msg->mask & IMC::OperationalLimits::OPL_MAX_DEPTH)
			obs.restrictAttribute("maxDepth", FloatDomain(msg->max_depth));

		if ((msg->mask & IMC::OperationalLimits::OPL_MAX_ALT))
			obs.restrictAttribute("maxAltitude", FloatDomain(msg->max_depth));
		if(msg->mask & IMC::OperationalLimits::OPL_MIN_ALT)
			obs.restrictAttribute("minAltitude", FloatDomain(msg->min_altitude));

		if (msg->mask & IMC::OperationalLimits::OPL_MAX_SPEED)
			obs.restrictAttribute("maxSpeed", FloatDomain(msg->max_speed));
		if (msg->mask & IMC::OperationalLimits::OPL_MIN_SPEED)
			obs.restrictAttribute("minSpeed", FloatDomain(msg->min_speed));

		oplimits_posted = true;
		postObservation(obs);
	}
	else
	{

		if (!oplimits_posted)
			postObservation(
					Observation("oplimits", "undefined"));

		if (!aggregate.count(IMC::OperationalLimits::getIdStatic()))
		{
			IMC::GetOperationalLimits get;
			sendMsg(get);
		}

	}

	/* VEHICLE_STATE */

	if (received.count(IMC::VehicleState::getIdStatic()))
	{
		IMC::VehicleState * msg =
				dynamic_cast<IMC::VehicleState *>(aggregate[IMC::VehicleState::getIdStatic()]);
		std::string mode;
		switch (msg->op_mode)
		{
		case IMC::VehicleState::VS_MANEUVER:
			mode = "Exec";
			break;
		case IMC::VehicleState::VS_SERVICE:
			mode = "Ready";
			break;
		case IMC::VehicleState::VS_CALIBRATION:
			mode = "Boot";
			break;
		case IMC::VehicleState::VS_EXTERNAL:
			mode = "Boot";
			break;
		case IMC::VehicleState::VS_ERROR:
			mode = "Error";
			break;
		default:
			std::ostringstream ss;
			ss << "Unknown vehicle mode: " << msg->op_mode;
			reportToDune(ss.str());
			syslog("WARN") << ss.str();
			mode = "undefined";
			break;
		}
		if (last_vstate != msg->op_mode)
		{
			postObservation(Observation("vstate", mode));
			if (msg->op_mode != IMC::VehicleState::VS_MANEUVER && !was_idle)
			{
				postObservation(Observation("maneuver", "Idle"));
				postObservation(Observation("command", "Idle"));
				was_idle = true;
			}
			else
				was_idle = false;
		}
		last_vstate = msg->op_mode;
	}
	else if (last_vstate == -1)
	{
		postObservation(
				Observation("vstate", "Boot"));
		if (!was_idle)
		{
			postObservation(Observation("maneuver", "Idle"));
			postObservation(Observation("command", "Idle"));
			was_idle = true;
		}
	}

	/* VEHICLE_COMMAND */

        int eta = std::numeric_limits<int>::max();
        
	if (received.count(IMC::VehicleState::getIdStatic()))
	{
		IMC::VehicleState *lastState =
				dynamic_cast<IMC::VehicleState*>(aggregate[IMC::VehicleState::getIdStatic()]);
		eta = lastState->maneuver_eta;
		if (lastState->op_mode == IMC::VehicleState::VS_MANEUVER
				&& !lastCommand.maneuver.isNull())
		{

			Observation obs("command", "Maneuver");
			IMC::Message * man = lastCommand.maneuver.get();

			obs.restrictAttribute("eta", IntegerDomain(eta));

			if (man->getId() == IMC::Goto::getIdStatic())
			{
				IMC::Goto * m = dynamic_cast<IMC::Goto *>(man);
				obs.restrictAttribute("latitude", FloatDomain(m->lat /*floor(n,2), ceil(n,2)*/));
				obs.restrictAttribute("longitude", FloatDomain(m->lon /*floor(e,2), ceil(e,2)*/));
				obs.restrictAttribute("depth", FloatDomain(m->z));
				obs.restrictAttribute("speed", FloatDomain(m->speed));
				obs.restrictAttribute("secs", IntegerDomain(0));
			}
			else if (man->getId() == IMC::Loiter::getIdStatic())
			{

				IMC::Loiter * m = dynamic_cast<IMC::Loiter *>(man);
				//double n, e;
				obs.restrictAttribute("latitude", FloatDomain(m->lat /*floor(n,2), ceil(n,2)*/));
				obs.restrictAttribute("longitude", FloatDomain(m->lon /*floor(e,2), ceil(e,2)*/));
				obs.restrictAttribute("depth", FloatDomain(m->z));
				obs.restrictAttribute("speed", FloatDomain(m->speed));
				obs.restrictAttribute("secs", IntegerDomain(m->duration));
			}
			else if (man->getId() == IMC::StationKeeping::getIdStatic())
			{
				IMC::StationKeeping * m = dynamic_cast<IMC::StationKeeping *>(man);
				//double n, e;
				obs.restrictAttribute("latitude", FloatDomain(m->lat /*floor(n,2), ceil(n,2)*/));
				obs.restrictAttribute("longitude", FloatDomain(m->lon /*floor(e,2), ceil(e,2)*/));
				obs.restrictAttribute("depth", FloatDomain(0));
				obs.restrictAttribute("speed", FloatDomain(m->speed));
				obs.restrictAttribute("secs", IntegerDomain(m->duration));
			}
			else if (man->getId() == IMC::Teleoperation::getIdStatic())
			{
				//  obs.restrictAttribute("type", TREX::europa::EuropaEntity("teleop"));
				IMC::EstimatedState * msg =
						dynamic_cast<IMC::EstimatedState *>(aggregate[IMC::EstimatedState::getIdStatic()]);
				//double lat = msg->lat, lon = msg->lon, hae = 0.0, northing, easting;
				obs.restrictAttribute("latitude", FloatDomain(msg->lat /*floor(n,2), ceil(n,2)*/));
				obs.restrictAttribute("longitude", FloatDomain(msg->lon /*floor(e,2), ceil(e,2)*/));
				obs.restrictAttribute("depth", FloatDomain(0));
				obs.restrictAttribute("speed", FloatDomain(0));
				obs.restrictAttribute("secs", IntegerDomain(0));
			}
			lastCommand.maneuver = IMC::InlineMessage();
			postObservation(obs);

		}
	}
}

bool
Platform::sendMsg(Message& msg, Address &dest)
{
	Dune::Utils::ByteBuffer bb;
	try
	{
		msg.setTimeStamp();
		msg.serialize(bb);
		send.write((const char*)bb.getBuffer(), msg.getSerializationSize(), dest,
				duneport);
	}
	catch (std::runtime_error& e)
	{
		syslog("LSTS") << "ERROR: " << e.what();
		return false;
	}
	return true;
}

bool
Platform::sendMsg(Message& msg, std::string ip, int port)
{
	Dune::Utils::ByteBuffer bb;
	try
	{
		msg.setTimeStamp();
		msg.setSource(65000);
		msg.setDestination(remote_id);
		msg.serialize(bb);
		send.write((const char*)bb.getBuffer(), msg.getSerializationSize(),
				Address(ip.c_str()), port);
	}
	catch (std::runtime_error& e)
	{
		syslog("LSTS") << "ERROR: " << e.what();
		return false;
	}
	return true;
}

bool
Platform::sendMsg(Message& msg)
{
	return sendMsg(msg, duneip, duneport);
}

bool Platform::commandManeuver(const std::string &man_name, IMC::Message * maneuver) {
	IMC::PlanControl pcontrol;

	pcontrol.arg.set(maneuver);
	pcontrol.info = "Maneuver initiated from TREX";
	pcontrol.op = IMC::PlanControl::PC_START;
	pcontrol.type = IMC::PlanControl::PC_REQUEST;
	pcontrol.plan_id = man_name;
	pcontrol.flags = 0;

	return sendMsg(pcontrol);
}

bool Platform::reportToDune(const std::string &message)
{
	return reportToDune(IMC::LogBookEntry::LBET_INFO, message);
}

bool Platform::reportToDune(int type, const std::string &message)
{
	IMC::LogBookEntry entry;
	entry.text = message;
	entry.context = "Autonomy.TREX";
	entry.htime = Time::Clock::getSinceEpoch();
	entry.type = type;
	return sendMsg(entry);
}

bool Platform::reportErrorToDune(const std::string &message)
{
	return reportToDune(IMC::LogBookEntry::LBET_ERROR, message);
}

IMC::Message *
Platform::commandGoto(const std::string &man_name, double lat, double lon, double depth, double speed)
{
	IMC::Goto * msg = new IMC::Goto();
	msg->lat = lat;
	msg->lon = lon;
	msg->z = depth;
	msg->speed = speed;
	msg->speed_units = IMC::Goto::SUNITS_METERS_PS;

	if (commandManeuver(man_name, msg))
		return msg;
	else
		return NULL;
}


IMC::Message *
Platform::commandStationKeeping(const std::string &man_name, double lat, double lon, double speed,
		int seconds)
{
	IMC::StationKeeping * msg = new IMC::StationKeeping();
	msg->lat = lat;
	msg->lon = lon;
	msg->duration = seconds;
	msg->radius = 15;
	msg->speed = speed;
	msg->speed_units = IMC::StationKeeping::SUNITS_METERS_PS;

	if (commandManeuver(man_name, msg))
		return msg;
	else
		return NULL;
}

IMC::Message *
Platform::commandLoiter(const std::string &man_name, double lat, double lon, double depth, double radius,
		double speed, int seconds)
{
	IMC::Loiter * msg = new IMC::Loiter();
	msg->lat = lat;
	msg->lon = lon;
	msg->z = depth;
	msg->duration = seconds;
	msg->radius = radius;
	msg->direction = IMC::Loiter::LD_CLOCKW;
	msg->speed = speed;
	msg->speed_units = IMC::Loiter::SUNITS_METERS_PS;

	if (commandManeuver(man_name, msg))
		return msg;
	else
		return NULL;
}

IMC::Message *
Platform::commandIdle(const std::string &man_name)
{

	IMC::IdleManeuver * msg = new IMC::IdleManeuver();

	if (commandManeuver(man_name, msg))
		return msg;
	else
		return NULL;
}

