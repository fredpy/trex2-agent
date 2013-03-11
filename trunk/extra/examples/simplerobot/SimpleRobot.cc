#include <iostream>

#include <trex/domain/StringDomain.hh>

#include "SimpleRobot.hh"

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::simpleRobot;

Symbol const SimpleRobot::AtPred("At");
Symbol const SimpleRobot::GoingPred("Going");
Symbol const SimpleRobot::locationPred("Location");
Symbol const SimpleRobot::pathPred("Path");

Symbol const SimpleRobot::navigatorObj("rover.navigator");

SimpleRobot::SimpleRobot(TeleoReactor::xml_arg_type arg)
    :TeleoReactor(arg,false), m_firstTick(true)
{
    syslog()<<"I want to own "<<navigatorObj;
    provide(navigatorObj, false); // declare the navigator timeline --> interna
    provide(navigatorObj); // declare the navigator timeline --> externa
    syslog()<<"I am done";
}

SimpleRobot::~SimpleRobot() {}

void SimpleRobot::handleInit()
{
    location = "Unknown";
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
