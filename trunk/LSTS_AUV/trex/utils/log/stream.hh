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
#ifndef H_trex_utils_log_stream
# define H_trex_utils_log_stream

# include "log_fwd.hh"
# include "bits/log_stream.hh"
# include "../platform/memory.hh"
# include "../platform/cpp11_deleted.hh"

# include <iostream>
# include <memory>

namespace TREX {
  namespace utils {
    namespace log {
      
      /** @brief Log entry constrcution stream
       *
       * A C++ stream that allows to generate new log entries 
       *
       * @note While this class implements multiple constructors 
       * as public, none of them are meant to be used explicitely. 
       * Indeedd this type is just used as proxy to control the 
       * lifetime of the entry construction implicitely. And should 
       * be used only implictely as the result of the @c text_log::msg
       * methods. For example it should be used as follow:
       * @code
       *   using namespace TREX::utils;
       *   // ...
       *   log::text_log my_log;
       *   // ...
       *   // implicit use of log::stream
       *   my_log("me", log::info)<<"This my log message."<<std::endl;
       * @end_code
       * while the following, even though syntactically correct, should be avoided :
       * @code 
       *   using namespace TREX::utils;
       *   // ...
       *   log::text_log my_log;
       *   // ...
       *   // explicit use of log::stream
       *   log::stream msg(my_log("me", log::info));
       *   msg<<"This my log message."<<std::endl;
       * @endcode 
       * Indeed on the later code The message won't be dispatched until @c msg 
       * is destroyed which depends on the remaining of implementation. On the 
       * other hand the former code uses log::stream as an implicit local varaible
       * that will be destroyed as soon as not used (meaning as soons as the 
       * message has been written in the stream) resulting on the dispatch of 
       * the message as early as intended.
       *
       * @relates text_log
       * @ingroup utils
       */
      class stream {
      public:
        /** @brief Copy constructor
         *
         * @param[in] other Another instance 
         * 
         * Tranfert the entry ownership from @p other to this ne instance
         */
        stream(stream const &other):m_out(STD_MOVE(other.m_out)) {}
        /** @brief Destructor */
        ~stream() {}
        
        /** @{
         * @brief base stream access
         *
         * utiliy methodds to access to the real stream implementation
         *
         * @return A stancdarad C++ output stream
         */
        std::ostream &get_stream();
        operator std::ostream &() {
          return get_stream();
        }
        /** @} */
        /** @brief Stream output operator
         *
         * @tparam Ty An output streamable type
         * @param[in] val An object ot write
         *
         * Write he value @p val into this entry (if the entry is valid)
         *
         * @return the subjacent output stream
         *
         * @note This method is strictly equivalent to 
         * @code 
         *  get_stream()<<val
         * @endcode
         */
        template<typename Ty>
        std::ostream &operator<<(Ty const &val) {
          return get_stream()<<val;
        }
        
      private:
        explicit stream(details::entry_sink const &dest);
        
        mutable UNIQ_PTR<details::stream_impl> m_out;
        
        friend class text_log;
# ifndef DOXYGEN
        stream() DELETED; // Non default constructible
# endif // DOXYGEN
      };

    } // TREX::utils::log
  } // TREX::utils
} // TREX

#endif // H_trex_utils_log_stream
