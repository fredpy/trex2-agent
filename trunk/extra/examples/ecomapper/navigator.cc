#include "navigator.hh"
#include "ros_listener.hh"

#include <trex/domain/FloatDomain.hh>
#include <iostream>

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::ecomapper;

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
	
}
void Navigator::ctd_rhObservation(TREX::transaction::Observation const &obs)
{

}
void Navigator::fixObservation(TREX::transaction::Observation const &obs)
{
	lock.lock();
	//FloatDomain const & x= obs.getAttribute(Symbol("latitude")).typedDomain();
	//FloatDomain y = obs.getDomain(Symbol("longitude"));
	//This is were the point is add to the spline if depth is deep enough
	//spline.addPoint(x,y);
	lock.unlock(); 
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

