#include <iostream>

#include <trex/domain/StringDomain.hh>

#include "Rover.hh"

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::rover;

Symbol const Rover::AtPred("At");
Symbol const Rover::GoingPred("Going");
Symbol const Rover::locationPred("Location");
Symbol const Rover::pathPred("Path");
Symbol const Rover::StowedPred("Stowed");
Symbol const Rover::StowingPred("Stowing");
Symbol const Rover::UnstowedPred("Unstowed");
Symbol const Rover::UnstowingPred("Unstowing");
Symbol const Rover::PlacedPred("Placed");
Symbol const Rover::SamplingPred("Sampling");
Symbol const Rover::FreePred("Free");

Symbol const Rover::navigatorObj("Navigator");
Symbol const Rover::instrumentLocationObj("InstrumentLocation");
Symbol const Rover::instrumentStateObj("InstrumentState");

Rover::Rover(TeleoReactor::xml_arg_type arg)
    :TeleoReactor(arg,false), m_firstTick(true)
{
    syslog()<<"I want to own "<<navigatorObj;
    provide(navigatorObj); // declare the navigator timeline
    syslog()<<"I want to own "<<instrumentLocationObj;
    provide(instrumentLocationObj); // declare the instrumentLocation timeline
    syslog()<<"I want to own "<<instrumentStateObj;
    provide(instrumentStateObj); // declare the instrumentState timeline
    syslog()<<"I am done";
}

Rover::~Rover() {}

void Rover::handleInit()
{
    location = "Unknown";
}

void Rover::setValue(TREX::transaction::goal_id const &g)
{
    if(g->predicate()==AtPred)
    {
        location = g->getAttribute(locationPred).domain().getStringSingleton();
        m_navigator_state.reset(new Observation(navigatorObj, AtPred));
        (*m_navigator_state).restrictAttribute(locationPred, StringDomain(location));
        postObservation(*m_navigator_state);
    } else if(g->predicate()==GoingPred) {
        location = g->getAttribute(pathPred).domain().getStringSingleton();
        m_navigator_state.reset(new Observation(navigatorObj, GoingPred));
        (*m_navigator_state).restrictAttribute(pathPred, StringDomain(location));
        postObservation(*m_navigator_state);
    } else if(g->predicate()==StowedPred || g->predicate()==StowingPred ||
              g->predicate()==UnstowedPred || g->predicate()==UnstowingPred)
    {
        m_InstrumentLocation_state.reset(new Observation(instrumentLocationObj,g->predicate()));
        postObservation(*m_InstrumentLocation_state);
    } else if(g->predicate()==PlacedPred || g->predicate()==SamplingPred
              || g->predicate()==FreePred)
    {
        m_Instrument_state.reset(new Observation(instrumentStateObj,g->predicate()));
        postObservation(*m_Instrument_state);
    }
}

bool Rover::synchronize()
{
    if(m_firstTick)
    {
        m_navigator_state.reset(new Observation(navigatorObj, AtPred));
        (*m_navigator_state).restrictAttribute(locationPred, StringDomain(location));
        postObservation(*m_navigator_state);
        m_InstrumentLocation_state.reset(new Observation(instrumentLocationObj, StowedPred));
        postObservation(*m_InstrumentLocation_state);
        m_Instrument_state.reset(new Observation(instrumentStateObj, FreePred));
        postObservation(*m_Instrument_state);
        m_firstTick = false;
        m_nextTick = getCurrentTick()+1;
    } else {
        TICK cur = getCurrentTick();
        if(m_nextTick<=cur)
        {
            while(!m_pending.empty())
            {
                if(m_pending.front()->startsAfter(cur))
                {
                    if(m_pending.front()->startsBefore(cur))
                    {
                        setValue(m_pending.front());
                        m_nextTick = cur+m_pending.front()->getDuration().lowerBound().value();
                        m_pending.pop_front();
                    }
                    break;
                } else {
                    m_pending.pop_front();
                }
            }
        }
    }
    return true;
}

void Rover::handleRequest(goal_id const &g)
{
    if(g->predicate()==AtPred || g->predicate()==GoingPred || g->predicate()==PlacedPred
       || g->predicate()==SamplingPred || g->predicate()==FreePred)
    {
        IntegerDomain::bound lo = g->getStart().lowerBound();
        if(lo.isInfinity())
        {
            m_pending.push_front(g);
        } else {
            std::list<goal_id>::iterator i = m_pending.begin();
            TICK val = lo.value();
            for(; i!=m_pending.end(); ++i)
            {
                if((*i)->startsAfter(val))
                    break;
            }
            m_pending.insert(i,g);
        }
    }
}

void Rover::handleRecall(goal_id const &g)
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
