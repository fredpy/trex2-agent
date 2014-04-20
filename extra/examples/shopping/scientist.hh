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
#ifndef H_Scientist
#define H_Scientist

#include <trex/transaction/TeleoReactor.hh>

namespace TREX {
  /** @brief AgentLocation plug-in utilities
   *
   * This namespace embeds the classes and functions provided by the
   * AgentLocation plug-in
   *
   * @ingroup AgentLocation
   *
   */
  namespace Scientist {

    class Scientist :public TREX::transaction::TeleoReactor {
    public:
      /** @brief XML constructor
       * @param arg An XML node definition
       *
       * This constructor is called to generate a location reactor
       * based on an XML node. The expected XML format is the following:
       * @code
       * <Scientist name="<name>" latency="<int>" lookahead="<int>" />
       * @endcode
       */
      Scientist(TREX::transaction::TeleoReactor::xml_arg_type arg);
      /** @brief Destructor */
      ~Scientist();

    private:
      void handleInit();
      bool synchronize();

      /** @brief Name of the timeline auv */
      static TREX::utils::Symbol const auv;
      /** @brief Name of the predicate Sample */
      static TREX::utils::Symbol const Sample;

      /** @brief Name of the pred for Locations */
      static TREX::utils::Symbol const Objective;
      /** @brief Name of the Locations */
      static std::string const Vent1;
      static std::string const Vent2;

      // int number;

    };

  }
}

#endif // H_Scientist
