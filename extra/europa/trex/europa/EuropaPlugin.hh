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
# include "config.hh"

# include <PLASMA/CFunction.hh>

namespace TREX {
  namespace europa {

    class Assembly;

    namespace details {
      class Schema;
    } // TREX::europa::details

    /** @brief Europa extension manager
     * 
     * This base class is used to add europa extensions to T-REX 
     * Assembly.
     * 
     * When a new Assembly is created it will look to all the instances of this 
     * class currently existing and will call @c registerComponents for all of 
     * these passing itself as an argument. 
     * 
     * This mechanism along with the EUROPA macro adaptations to T-REX allow 
     * to augment the Assembly with new europa extensions such as :
     * @li Constraints using @c TREX_REGISTER_CONSTRAINT
     * @li Flaw filters using @c TREX_REGISTER_FLAW_FILTER
     * @li Flaw handlers using @c TREX_REGISTER_FLAW_HANDLER
     * @li Flaw managers using @c TREX_REGISTER_FLAW_MANAGER
     * @li New components using @c TREX_REGISTER_COMPONENT_FACTORY
     * @li New matches using @c TREX_REGISTER_MATCH_FINDER
     *
     * All of these macro have the same behavior as their europa counterpart 
     * (which are not prefixed by @c TREX_) except they specialize to apply to 
     * the specific Assembly passed as argument.
     * 
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup europa
     */ 
    class EuropaPlugin {
    public:
      /** @brief Destructor */
      virtual ~EuropaPlugin();

    protected:
      /** @brief Constructor */
      EuropaPlugin();
      /** @brief plugin registration to an Assembly
       *
       * @param[in,out] assembly A europa assembly
       *
       * This callback is called whenever a new Assembly @p assembly is 
       * created during the liufetime of this instance. I tallows programmer 
       * through this call to inject Europa extension within this assembly.
       */
      virtual void registerComponents(Assembly const &assembly) =0;
      /** @brief Declare a function ot Europa
       *
       * @param[in,out] assembly The target assembly
       * @param[in] fn A pointer to the function object
       *
       * An utility to ease the decalration of a new Europa function @p fn to
       * the Assembly @p assembly. A europa function is a syntactic sugar for 
       * nddl that allows a constraint preficate to be mapped as a function
       * 
       * @post The function @p fn is now understood by the constraint engine 
       * of @p assembly
       */
      void declareFunction(Assembly const &assembly, EUROPA::CFunction *fn);

    private:
      TREX::utils::SingletonUse<details::Schema> m_schema;

      friend class details::Schema;
    }; // TREX::europa::EuropaPlugin

  } // TREX::europa
} // TREX

/** @brief Add new constraint type to an Assembly
 * 
 * @param[in] assembly   An assembly
 * @param[in] class_name The name of the class to be used for this constraint
 * @param[in] label      The symbolic name associated to this constraint
 * @param[in] propagator The propagator that will handle this constraint
 * 
 * @relates TREX::europa::EuropaPlugin
 * @ingroup europa
 */
#define TREX_REGISTER_CONSTRAINT(assembly, class_name, label, propagator)\
  {  \
    EUROPA::ConstraintEngine* ce = (EUROPA::ConstraintEngine*) assembly.getComponent("ConstraintEngine"); \
    using EUROPA::ConcreteConstraintType; \
    using EUROPA::LabelStr; \
    REGISTER_CONSTRAINT(ce->getCESchema(), class_name, #label, #propagator);\
  }

/** @brief Add new flaw filter to an Assembly
 * 
 * @param[in] assembly   An assembly
 * @param[in] class_name The name of the class to be used for this flaw filter
 * @param[in] label      The name used for this filter
 * 
 * @relates TREX::europa::EuropaPlugin
 * @ingroup europa
 */
# define TREX_REGISTER_FLAW_FILTER(assembly, class_name, label) \
  REGISTER_FLAW_FILTER(((EUROPA::SOLVERS::ComponentFactoryMgr*)assembly.getComponent("ComponentFactoryMgr")), class_name, label);

/** @brief Add new flaw handler to an Assembly
 * 
 * @param[in] assembly   An assembly
 * @param[in] class_name The name of the class to be used for this flaw handler
 * @param[in] label      The name used for this handler
 * 
 * @relates TREX::europa::EuropaPlugin
 * @ingroup europa
 */
# define TREX_REGISTER_FLAW_HANDLER(assembly, class_name, label) \
  REGISTER_FLAW_HANDLER(((EUROPA::SOLVERS::ComponentFactoryMgr*)assembly.getComponent("ComponentFactoryMgr")), class_name, label);

/** @brief Add new flaw manager to an Assembly
 * 
 * @param[in] assembly   An assembly
 * @param[in] class_name The name of the class to be used for this flaw manager
 * @param[in] label      The name used for this manager
 * 
 * @relates TREX::europa::EuropaPlugin
 * @ingroup europa
 */
# define TREX_REGISTER_FLAW_MANAGER(assembly, class_name, label) \
  REGISTER_FLAW_MANAGER(((EUROPA::SOLVERS::ComponentFactoryMgr*)assembly.getComponent("ComponentFactoryMgr")), class_name, label);

/** @brief Add new component type to an Assembly
 * 
 * @param[in] assembly   An assembly
 * @param[in] class_name The name of the class to manage this component
 * @param[in] label      The name used for this component
 * 
 * @relates TREX::europa::EuropaPlugin
 * @ingroup europa
 */
# define TREX_REGISTER_COMPONENT_FACTORY(assembly, class_name, label) \
  REGISTER_COMPONENT_FACTORY(((EUROPA::SOLVERS::ComponentFactoryMgr*)assembly.getComponent("ComponentFactoryMgr")), class_name, label);

/** @brief Add new Match finder to an Assembly
 * 
 * @param[in] assembly   An assembly
 * @param[in] class_name The name of the class to be used for finding matches
 * @param[in] label      The name used for this component
 * 
 * @relates TREX::europa::EuropaPlugin
 * @ingroup europa
 */
# define TREX_REGISTER_MATCH_FINDER(assembly, class_name, label) \
  REGISTER_MATCH_FINDER(((EUROPA::SOLVERS::MatchFinderMgr*)assembly.getComponent("MatchFinderMgr")), class_name, label)


/** @brief Convert A C++ litterate to a string
 * 
 * @param[in] symbol   A C++ litterate
 * @return the string corresponding to @p symbol
 * @sa TO_STRING_EVAL
 * @ingroup europa
 */
# define TO_STRING(symbol) #symbol
/** @brief Convert A C++ litterate to a string
 * 
 * @param[in] symbol   A C++ litterate or a cpp macro
 * @return the string corresponding to @p symbol after cpp macro expansion of 
 *         @p symbol
 * @sa TO_STRING
 * @ingroup europa
 */
# define TO_STRING_EVAL(symbol) TO_STRING(symbol)

#endif // H_trex_EuropaPlugin