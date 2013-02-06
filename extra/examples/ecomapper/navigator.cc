#include "navigator.hh"
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
    use("Latitude");
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
    std::cout<<"Recieved!"<<std::endl;
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

