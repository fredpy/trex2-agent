#include "navigator.hh"
#include "ros_listener.hh"

#include <trex/domain/FloatDomain.hh>
#include <iostream>

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::ecomapper;

static const std::string gLatitude("latitude"), gLongitude("longitude"),
                         gDepth("depth"), gAltitude("altitude"),
                         gTotal_Water_Column("Total_Water_Column");
static const double gDepthThreshold(100);

Navigator::Navigator(TeleoReactor::xml_arg_type arg)
    :TeleoReactor(arg,false)
{

}

Navigator::~Navigator() {}

void Navigator::handleInit()
{
    use(Ros_Listener::dvlObj);
    use(Ros_Listener::ctd_rhObj);
    use(Ros_Listener::fixObj);
}

bool Navigator::synchronize()
{
    TICK cur = getCurrentTick();
    if(m_nextTick<=cur)
    {
        while(!m_pending.empty())
        {
            if(m_pending.front()->startsAfter(cur))
            {
                if(m_pending.front()->startsBefore(cur))
                {
                    //setValue(m_pending.front());
                    m_nextTick = cur+m_pending.front()->getDuration().lowerBound().value();
                    m_pending.pop_front();
                }
                break;
            } else {
                m_pending.pop_front();
            }
        }
    }
    return true;
}

void Navigator::notify(TREX::transaction::Observation const &obs)
{
	///Dispatching the different timeline observations to their respective function
	TREX::utils::Symbol const & object = obs.object();
	if(object==Ros_Listener::dvlObj)
		dvlObservation(obs);
	else if(object==Ros_Listener::ctd_rhObj)
		ctd_rhObservation(obs);
	else if(object==Ros_Listener::fixObj)
		fixObservation(obs);
}

void Navigator::dvlObservation(TREX::transaction::Observation const &obs)
{
    lock.lock();
    double depth = getAttribute(gDepth, obs);
    double altitude = getAttribute(gAltitude, obs);
    addToMap(gDepth, depth);
    addToMap(gAltitude, altitude);
    //Still need to find the actual total_water_column
    addToMap(gTotal_Water_Column,depth+altitude);
    lock.unlock();
}
void Navigator::ctd_rhObservation(TREX::transaction::Observation const &obs)
{

}
void Navigator::fixObservation(TREX::transaction::Observation const &obs)
{
	lock.lock();
	double x = getAttribute(gLatitude, obs);
    double y = getAttribute(gLongitude, obs);
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
	lock.unlock();
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

