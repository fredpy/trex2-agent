/*
 * DummyOperator.cc
 *
 *  Created on: Apr 29, 2013
 *      Author: zp
 */

#include "extra/third_party/lsts/DummyOperator.hh"
# include "Platform.hh"
using namespace TREX::LSTS;
using namespace TREX::transaction;
using DUNE_NAMESPACES;

namespace
{

  /** @brief DummyOperator reactor declaration */
  TeleoReactor::xml_factory::declare<SafetyBug> decl("DummyOperator");

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


  }
}

void DummyOperator::handleTickStart()
{

}

void DummyOperator::handleInit()
{

}

bool DummyOperator::synchronize()
{
  return true;
}

