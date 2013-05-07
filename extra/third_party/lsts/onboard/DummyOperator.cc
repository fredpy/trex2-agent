/*
 * DummyOperator.cc
 *
 *  Created on: Apr 29, 2013
 *      Author: zp
 */

# include "DummyOperator.hh"
# include "Platform.hh"
using namespace TREX::LSTS;
using namespace TREX::transaction;
using DUNE_NAMESPACES;

namespace
{

  /** @brief DummyOperator reactor declaration */
  TeleoReactor::xml_factory::declare<DummyOperator> decl("DummyOperator");

}

DummyOperator::DummyOperator(TREX::transaction::TeleoReactor::xml_arg_type arg)
:TeleoReactor(arg)
{

}

DummyOperator::~DummyOperator() {
  // TODO Auto-generated destructor stub
}

void DummyOperator::notify(TREX::transaction::Observation const &obs)
{
  if (obs.object().str() == "control" && obs.predicate() == "DUNE")
  {
    Platform *r = m_env->getPlatformReactor();
    // TODO activate TREX

    PlanControl pc;
    pc.op = PlanControl::PC_STOP;
    pc.info = "TREX Dummy Operator Reactor";
    m_env.instance().getPlatformReactor()->sendMsg(pc);

    SetEntityParameters params;
    params.name = "TREX";
    EntityParameter param;
    param.name = "Active";
    param.value = "true";
    params.params.push_back(param);
    m_env.instance().getPlatformReactor()->sendMsg(params);
  }

}
