#include <iostream>

#include <trex/domain/string_domain.hh>

#include "Rover.hh"

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::rover;

symbol const Rover::AtPred("At");
symbol const Rover::GoingPred("Going");
symbol const Rover::locationPred("Location");
symbol const Rover::pathPred("Path");
symbol const Rover::StowedPred("Stowed");
symbol const Rover::StowingPred("Stowing");
symbol const Rover::UnstowedPred("Unstowed");
symbol const Rover::UnstowingPred("Unstowing");
symbol const Rover::PlacedPred("Placed");
symbol const Rover::SamplingPred("Sampling");
symbol const Rover::FreePred("Free");

symbol const Rover::navigatorObj("Navigator");
symbol const Rover::instrumentLocationObj("InstrumentLocation");
symbol const Rover::instrumentStateObj("InstrumentState");

Rover::Rover(reactor::xml_arg_type arg)
    :reactor(arg,false), m_firstTick(true)
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

void Rover::handle_init()
{
    location = "Unknown";
}

void Rover::setValue(TREX::transaction::token_id const &g)
{
    if(g->predicate()==AtPred)
    {
        location = g->attribute(locationPred).domain().get_singleton_as_string();
        m_navigator_state.reset(new token(navigatorObj, AtPred));
        (*m_navigator_state).restrict_attribute(locationPred, string_domain(location));
        post_observation(*m_navigator_state);
    } else if(g->predicate()==GoingPred) {
        location = g->attribute(pathPred).domain().get_singleton_as_string();
        m_navigator_state.reset(new token(navigatorObj, GoingPred));
        (*m_navigator_state).restrict_attribute(pathPred, string_domain(location));
        post_observation(*m_navigator_state);
    } else if(g->predicate()==StowedPred || g->predicate()==StowingPred ||
              g->predicate()==UnstowedPred || g->predicate()==UnstowingPred)
    {
        m_InstrumentLocation_state.reset(new token(instrumentLocationObj,g->predicate()));
        post_observation(*m_InstrumentLocation_state);
    } else if(g->predicate()==PlacedPred || g->predicate()==SamplingPred
              || g->predicate()==FreePred)
    {
        m_Instrument_state.reset(new token(instrumentStateObj,g->predicate()));
        post_observation(*m_Instrument_state);
    }
}

bool Rover::synchronize()
{
    if(m_firstTick)
    {
        m_navigator_state.reset(new token(navigatorObj, AtPred));
        (*m_navigator_state).restrict_attribute(locationPred, string_domain(location));
        post_observation(*m_navigator_state);
        m_InstrumentLocation_state.reset(new token(instrumentLocationObj, StowedPred));
        post_observation(*m_InstrumentLocation_state);
        m_Instrument_state.reset(new token(instrumentStateObj, FreePred));
        post_observation(*m_Instrument_state);
        m_firstTick = false;
        m_nextTick = current_tick()+1;
    } else {
        TICK cur = current_tick();
        if(m_nextTick<=cur)
        {
            while(!m_pending.empty())
            {
                if(m_pending.front()->starts_after(cur))
                {
                    if(m_pending.front()->starts_before(cur))
                    {
                        setValue(m_pending.front());
                        m_nextTick = cur+m_pending.front()->duration().lower_bound().value();
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

void Rover::handle_request(token_id const &g)
{
    if(g->predicate()==AtPred || g->predicate()==GoingPred || g->predicate()==PlacedPred
       || g->predicate()==SamplingPred || g->predicate()==FreePred)
    {
        int_domain::bound lo = g->start().lower_bound();
        if(lo.is_infinity())
        {
            m_pending.push_front(g);
        } else {
            std::list<token_id>::iterator i = m_pending.begin();
            TICK val = lo.value();
            for(; i!=m_pending.end(); ++i)
            {
                if((*i)->starts_after(val))
                    break;
            }
            m_pending.insert(i,g);
        }
    }
}

void Rover::handle_recall(token_id const &g)
{
    std::list<token_id>::iterator i = m_pending.begin();
    for(; i!=m_pending.end(); ++i)
    {
        if(*i==g)
        {
            m_pending.erase(i);
            return;
        }
    }
}
