/* -*- C++ -*- */
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
#ifndef H_trex_utils_log_out_file
# define H_trex_utils_log_out_file

# include "entry.hh"

# include <fstream>
# include <memory>

namespace TREX {
  namespace utils {
    namespace log {
      
      /** @brief Basic log file handler
       *
       * This class implements a basic log message handler for 
       * @c text_log that redirect all messages received in a 
       * file
       *
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup utils
       * @sa text_log
       */
      class out_file {
      public:
        typedef void           result_type;
        typedef entry::pointer argument_type;
        
        /** @brief Consrructor
         * @param[in] fname A file open
         *
         * Create a new instance that will redirect all messages 
         * received into the new ly created file @p fname
         */
        explicit out_file(std::string const &fname);
        /** @brief Destructor */
        ~out_file() {}
        
        /** @brief Message handling operator
         *
         * @param[in] msg A  new log entry
         *
         * Write @p msg in the output file
         */
        void operator()(entry::pointer msg);
        void flush();
        
      private:
        std::shared_ptr<std::ofstream> m_file;
      };
      
    }
  }
}

#endif // H_trex_utils_log_out_file
