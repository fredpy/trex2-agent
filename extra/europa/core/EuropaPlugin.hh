/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2011, MBARI.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TREX Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef H_trex_EuropaPlugin
# define H_trex_EuropaPlugin

# include <trex/utils/SingletonUse.hh>

# include "EuropaException.hh"

namespace TREX {
  namespace europa {

    class Assembly;

    namespace details {
      class Schema;
    } // TREX::europa::details

    class EuropaPlugin {
    public:
      virtual ~EuropaPlugin();

    protected:
      EuropaPlugin();
      virtual void registerComponents(Assembly const &assembly) =0;

    private:
      TREX::utils::SingletonUse<details::Schema> m_schema;

      friend class details::Schema;
    }; // TREX::europa::EuropaPlugin

  } // TREX::europa
} // TREX

#define TREX_REGISTER_CONSTRAINT(assembly, class_name, label, propagator)\
  {  \
    EUROPA::ConstraintEngine* ce = (EUROPA::ConstraintEngine*) assembly.getComponent("ConstraintEngine"); \
    using EUROPA::ConcreteConstraintType; \
    using EUROPA::LabelStr; \
    REGISTER_CONSTRAINT(ce->getCESchema(), class_name, #label, #propagator);\
  }

#define TREX_REGISTER_FLAW_FILTER(assembly, class_name, label) \
  REGISTER_FLAW_FILTER(((EUROPA::SOLVERS::ComponentFactoryMgr*)assembly.getComponent("ComponentFactoryMgr")), class_name, label);

#define TREX_REGISTER_FLAW_HANDLER(assembly, class_name, label) \
  REGISTER_FLAW_HANDLER(((EUROPA::SOLVERS::ComponentFactoryMgr*)assembly.getComponent("ComponentFactoryMgr")), class_name, label);

#define TREX_REGISTER_FLAW_MANAGER(assembly, class_name, label) \
  REGISTER_FLAW_MANAGER(((EUROPA::SOLVERS::ComponentFactoryMgr*)assembly.getComponent("ComponentFactoryMgr")), class_name, label);

#define TREX_REGISTER_COMPONENT_FACTORY(assembly, class_name, label) \
  REGISTER_COMPONENT_FACTORY(((EUROPA::SOLVERS::ComponentFactoryMgr*)assembly.getComponent("ComponentFactoryMgr")), class_name, label);

#define TREX_REGISTER_MATCH_FINDER(assembly, class_name, label) \
  REGISTER_MATCH_FINDER(((EUROPA::SOLVERS::MatchFinderMgr*)assembly.getComponent("MatchFinderMgr")), class_name, label)

#endif // H_trex_EuropaPlugin
