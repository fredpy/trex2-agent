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
  reactor::xml_factory::declare<DummyOperator> decl("DummyOperator");

}

DummyOperator::DummyOperator(TREX::transaction::reactor::xml_arg_type arg)
:reactor(arg)
{

}

DummyOperator::~DummyOperator() {
  // TODO Auto-generated destructor stub
}

void DummyOperator::notify(TREX::transaction::Observation const &obs)
{
  if (obs.object().str() == "control" && obs.predicate() == "DUNE")
  {
    // TODO activate TREX

    PlanControl pc;
    pc.op = PlanControl::PC_STOP;
    pc.info = "TREX Dummy Operator Reactor";
    m_env.instance().getPlatformReactor()->sendMsg(pc);
    std::cout << "Sending to Dune\n";
    pc.toJSON(std::cout);

    SetEntityParameters params;
    params.name = "TREX";
    EntityParameter param;
    param.name = "Active";
    param.value = "true";
    params.params.push_back(param);
    m_env.instance().getPlatformReactor()->sendMsg(params);
    params.toJSON(std::cout);

  }

}
