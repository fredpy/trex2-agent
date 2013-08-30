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
    :TeleoReactor(arg,false), ros_commanderBusy(true),
    beginBoundaryTracking(false), nextwp_number(2), numberOfSplinePoints(0)
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
        CoordinateMap::iterator coordinate = coordinates.find(currentTick);
        ///First equals latitude and second equals longitude
        if(!beginBoundaryTracking && wp_numbers[currentTick]==nextwp_number)
        {
            beginBoundaryTracking = true;
        }
        if(beginBoundaryTracking)
        {
            double currentWaypoint = wp_numbers[currentTick],
                   currentWaterColumn = columns[currentTick],
                   avgWaterColumn = 0, difference = 0;
            ///We are going back 10 ticks to check if they are in the same waypoint
            if(wp_numbers[currentTick-10]==currentWaypoint)
            {
                ///Average the values of 5 ticks starting 10 ticks before the current
                for(TICK t = currentTick-10; t<=currentTick-5; t++)
                {
                    avgWaterColumn += columns[t];
                }
                avgWaterColumn /= 5;
                difference = std::abs(currentWaterColumn - avgWaterColumn);
                ///If the difference is greater than value than we add it to the spline
                if(difference>1)
                {
                    nextwp_number++;
                    beginBoundaryTracking = false;
                    ///Add to spline
                    spline.addPoint(coordinate->second.first, coordinate->second.second);
                    numberOfSplinePoints++;
                }
            }

        }

        if(wp_numbers[currentTick]>=5 && numberOfSplinePoints>=3)
        {
            double longitude = spline(coordinate->second.first);
        }

        //std::fstream file;
        //file.open("test.csv", std::fstream::out | std::fstream::app);
        //std::pair<double, double>& coordinate = coordinates[currentTick];
        //file<<coordinate.first<<","<<coordinate.second<<",";
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
    lock.lock();
    if(obs.predicate()=="state")
    {
        double totalWaterColumn = getAttribute(gDepth, obs)+getAttribute(gAltitude, obs);
        if(totalWaterColumn>=999)
            columns[currentTick] = columns[currentTick-1];
        else
            columns[currentTick] = totalWaterColumn;
        double x = getAttribute(gLatitude, obs);
        double y = getAttribute(gLongitude, obs);
        coordinates[currentTick]= std::make_pair(x,y);
        wp_numbers[currentTick]= getAttribute("wp_number", obs);
    }
    lock.unlock();
}

void Navigator::dvlObservation(TREX::transaction::Observation const &obs)
{
    lock.lock();
    /*
    if(obs.predicate()=="dvl")
    {
        TickDoubleMap::iterator column;
        column = columns.find(currentTick);
        if(column!=columns.end())
        {
            column->second += getAttribute(gDepth, obs);
        } else {
            columns[currentTick]= getAttribute(gDepth, obs);
        }
    }
    */
    lock.unlock();
}

void Navigator::wqmObservation(TREX::transaction::Observation const &obs)
{
    lock.lock();
    /*
    if(obs.predicate()=="wqm")
    {
        TickDoubleMap::iterator column;
        column = columns.find(currentTick);
        if(column!=columns.end())
        {
            column->second += getAttribute(gDepth, obs);
        } else {
            columns[currentTick]= getAttribute(gDepth, obs);
        }
    }
    */
    lock.unlock();
}

void Navigator::ctd_rhObservation(TREX::transaction::Observation const &obs)
{

}

void Navigator::fixObservation(TREX::transaction::Observation const &obs)
{
	lock.lock();
	/*
	if(obs.predicate()=="extended_fix")
	{
        double x = getAttribute(gLatitude, obs);
        double y = getAttribute(gLongitude, obs);
        coordinates[currentTick]= std::make_pair(x,y);
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

