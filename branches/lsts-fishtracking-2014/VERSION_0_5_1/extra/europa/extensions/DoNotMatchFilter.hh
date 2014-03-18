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
#ifndef H_trex_europa_DoNotMatchFilter
# define H_trex_europa_DoNotMatchFilter

# include <trex/europa/config.hh>

// include plasma header as system files in order to disable warnings
# define TREX_PP_SYSTEM_FILE <PLASMA/FlawFilter.hh>
# include <trex/europa/bits/system_header.hh>

namespace TREX {
  namespace europa {

    /** @brief Negative filter for variables
     *
     * THis filter exclude all the europa entities except the ones ones  with a 
     * name that matches a defined set.
     * It is usefull to use a unbound variable manager to a specific set 
     * of variables
     *
     * @bug Thhis filter is purely experimental and should be repaced by a more 
     * general version in the future
     * @ingroup europa
     * @author Frederic Py 
     */
    class DoNotMatchFilter :public EUROPA::SOLVERS::FlawFilter {
    public:
      /** @brief Constructor
       *
       * @param[in] cfg XML configuration
       *
       * Create  an ew instance The format of @p cfg is expected to be :
       * @code
       * <FlawFilter component="doNotMatch">
       *   <Choice name="var1"/>
       *   <Choice name="var2"/>
       *   [...]
       * </FlawFilter>
       * @endcode
       * Where all the @c vari are the names of the varibales we do not want to 
       * be excluded by this filter
       */
      DoNotMatchFilter(EUROPA::TiXmlElement const &cfg);
      /** @brief Destructor */
      ~DoNotMatchFilter() {}
      
    private:
      bool test(EUROPA::EntityId const &entity);
      
      std::set<EUROPA::LabelStr> m_names;
    }; // TREX::europa::DoNotMatchFilter 
        
  } // TREX::europa
} // TREX


#endif // H_trex_europa_DoNotMatchFilter
