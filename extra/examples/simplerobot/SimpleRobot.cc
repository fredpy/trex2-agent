#include <iostream>

#include <trex/domain/string_domain.hh>

#include "SimpleRobot.hh"

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::simpleRobot;

// rover.navigator
// rover corresponds to "SimpleRobot rover" declared in SimpleRobot-initial-state.nddl
// navigator corresponds to the  Navigator navigator; declaration inside the SimpleRobot class in SimpleRobot-model.nddl
symbol const SimpleRobot::navigatorObj("rover.navigator");

SimpleRobot::SimpleRobot(TeleoReactor::xml_arg_type arg)
    :TeleoReactor(arg,false)
{
    syslog()<<"I want to own "<<navigatorObj;
    //provide(navigatorObj, false); // declare the navigator timeline --> internal
    // this is an external timeline
    // declare the navigator timeline (notice that the Navigator class in the nddl extends AgentTimeline)
    provide(navigatorObj);
    syslog()<<"I am done";
}

SimpleRobot::~SimpleRobot() {}

void SimpleRobot::handleInit()
{
}

void SimpleRobot::setValue(TREX::transaction::goal_id const &g)
{
	syslog()<<"setValue";
}

bool SimpleRobot::synchronize()
{
	syslog()<<"synchronize";
    return true;
}

void SimpleRobot::handleRequest(goal_id const &g)
{
	syslog()<<"handleRequest";
}

void SimpleRobot::handleRecall(goal_id const &g)
{
	syslog()<<"handleRecall";
}
