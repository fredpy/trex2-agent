/* -*- C++ -*- */
/** @file "trex/utils/log/log_fwd.hh" 
 * @brief Forward declaration of log utilities
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */
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
#ifndef FWD_trex_utils_log_log
# define FWD_trex_utils_log_log

# include "../Symbol.hh"

namespace TREX {
  namespace utils {
    /** @brief Logging utilities
     *
     * This namespace contains all the classes and utilities for 
     * logging text messages from T-REx activity.
     *
     * @ingroup utils
     */
    namespace log {
      
      /** @brief Message type identifier
       *
       * The type used to identify the type of log message
       * @ingroup utils
       */
      typedef Symbol id_type;
      class stream;
      
      extern id_type const null; //!< Untyped log message
      extern id_type const info; //!< Infromation log message
      extern id_type const warn; //!< Warning log message
      extern id_type const error; //!< Error log message
      
      class text_log;
      
      /** @brief Logging implementation details
       *
       * This namespace embeds classes and utilities used for implementation
       * details for the text logging mechanism used within TREX
       *
       * @ingroup utils
       */
      namespace details {
        
        class entry_sink;
        
      } // TREX::utils::log::details
    } // TREX::utils::log    
  } // TREX::utils
} // TREX

#endif // FWD_trex_utils_log_log
