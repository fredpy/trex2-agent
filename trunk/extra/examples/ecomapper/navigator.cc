#include "navigator.hh"
#include "ros_listener.hh"
#include "ros_commander.hh"

#include <trex/domain/FloatDomain.hh>
#include <iostream>
#include <fstream>

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::ecomapper;

Symbol const Navigator::navigatorObj("navigator");
Symbol const Navigator::waypointObj("waypoint");

static const std::string gLatitude("latitude"), gLongitude("longitude"),
                         gDepth("depth"), gAltitude("altitude");
static const double gDepthThreshold(100);

Navigator::Navigator(TeleoReactor::xml_arg_type arg)
    :TeleoReactor(arg,false), ros_commanderBusy(true)
{
    syslog()<<"I want to own "<<navigatorObj;
    provide(navigatorObj);
}

Navigator::~Navigator() {}

void Navigator::handleInit()
{
    currentTick = 0;
    use(Ros_Listener::stateObj);
    use(Ros_Listener::dvlObj);
    use(Ros_Listener::ctd_rhObj);
    use(Ros_Listener::fixObj);
    use(Ros_Listener::wqmObj);
    use(Ros_Listener::navSatFixObj);
    use(Ros_Commander::ros_commanderObj);
}

bool Navigator::synchronize()
{
    if(currentTick>=2)
    {
        //std::fstream file;
        //file.open("test.csv", std::fstream::out | std::fstream::app);
        //std::pair<double, double>& coordinate = coordinates[currentTick];
        //file<<coordinate.first<<","<<coordinate.second<<",";
        if(columns[currentTick]>=999)
        {
            columns[currentTick] = columns[currentTick-1];
        }
        //file<<columns[currentTick]<<std::endl;
        //file.close();
        if(!m_pending.empty() && !ros_commanderBusy)
        {
            goal_id& navGoal = m_pending.front();
            Goal goal = Goal(Ros_Commander::ros_commanderObj, waypointObj);
            double latit, longit;
            goal.restrictAttribute(navGoal->getAttribute(gLatitude));
            goal.restrictAttribute(navGoal->getAttribute(gLongitude));
            postGoal(goal);
            m_pending.pop_front();
        }
    }
    currentTick++;
    return true;
}

void Navigator::notify(TREX::transaction::Observation const &obs)
{
	///Dispatching the different timeline observations to their respective function
	TREX::utils::Symbol const & object = obs.object();
	if(object==Ros_Listener::stateObj)
        stateObservation(obs);
	else if(object==Ros_Listener::dvlObj)
		dvlObservation(obs);
    else if(object==Ros_Listener::wqmObj)
		wqmObservation(obs);
	else if(object==Ros_Listener::ctd_rhObj)
		ctd_rhObservation(obs);
	else if(object==Ros_Listener::fixObj)
		fixObservation(obs);
	else if(object==Ros_Listener::navSatFixObj)
		navSatFixObservation(obs);
    else if(object==Ros_Commander::ros_commanderObj)
        if(obs.predicate()=="PENDING" || obs.predicate()=="ACTIVE")
            ros_commanderBusy = true;
        else
            ros_commanderBusy = false;
}

void Navigator::stateObservation(TREX::transaction::Observation const &obs)
{

}

void Navigator::dvlObservation(TREX::transaction::Observation const &obs)
{
    lock.lock();
    if(obs.predicate()=="dvl")
    {
        WaterColumnMap::iterator column;
        column = columns.find(currentTick);
        if(column!=columns.end())
        {
            column->second += getAttribute(gDepth, obs);
        } else {
            columns[currentTick]= getAttribute(gDepth, obs);
        }
    }
    lock.unlock();
}

void Navigator::wqmObservation(TREX::transaction::Observation const &obs)
{
    lock.lock();
    if(obs.predicate()=="wqm")
    {
        WaterColumnMap::iterator column;
        column = columns.find(currentTick);
        if(column!=columns.end())
        {
            column->second += getAttribute(gDepth, obs);
        } else {
            columns[currentTick]= getAttribute(gDepth, obs);
        }
    }
    lock.unlock();
}

void Navigator::ctd_rhObservation(TREX::transaction::Observation const &obs)
{

}

void Navigator::fixObservation(TREX::transaction::Observation const &obs)
{
	lock.lock();
	if(obs.predicate()=="extended_fix")
	{
        double x = getAttribute(gLatitude, obs);
        double y = getAttribute(gLongitude, obs);
        coordinates[currentTick]= std::make_pair(x,y);
	}

    /*
    bool nextPoint = (getMapValue(gLatitude)!=0)?true:false;
    //If the depth is greater than the threshold add the inital point
	if(!nextPoint && getMapValue(gTotal_Water_Column) >= gDepthThreshold)
	{
	    addToMap(gLatitude, x);
        addToMap(gLongitude, y);
	}
	//If the dpeth is less than the treshold than we see
	//if we already have a value which means we are at the end of the depth
	else if(nextPoint)
	{
	    //We calculate the midpoint between the start and end points of the
	    //side of the canyon we just completed
	    double midx = (getMapValue(gLatitude)+x)/2;
	    double midy = (getMapValue(gLongitude)+y)/2;
	    //Add the point into the cubic spline
	    spline.addPoint(midx, midy);
	    //Then zero out the values
	    addToMap(gLatitude, 0);
	    addToMap(gLongitude, 0);
	}
	*/
	lock.unlock();
}

void Navigator::navSatFixObservation(TREX::transaction::Observation const &obs)
{

}


double Navigator::getAttribute(std::string const &name, Observation const &obs)
{
    double value;
    try
    {
        value = boost::any_cast<double>(obs.getAttribute(Symbol(name)).domain().getSingleton());
    } catch (boost::bad_any_cast& err) {
        value = -1;
    }
    return value;
}

void Navigator::handleRequest(goal_id const &g)
{
    if(g->predicate()==waypointObj)
    {
        m_pending.push_back(g);
    }
}

void Navigator::handleRecall(goal_id const &g)
{
    std::list<goal_id>::iterator i = m_pending.begin();
    for(; i!=m_pending.end(); ++i)
    {
        if(*i==g)
        {
            m_pending.erase(i);
            return;
        }
    }
}

