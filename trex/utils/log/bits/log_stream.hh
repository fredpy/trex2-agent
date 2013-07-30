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
#ifndef H_trex_utils_log_log_stream
# define H_trex_utils_log_log_stream

# include "log_sig.hh"

# include <boost/iostreams/stream.hpp>
# include "../../platform/memory.hh"

namespace TREX {
  namespace utils {
    namespace log {
      /** @brief Log implementation details
       *
       * This namespace includes implementation details for the log
       * management classes.
       *
       * @ingroup utils
       */
      namespace details {
      
        /** @brief Log enty construction manager
         *
         * This class is used to build and dispatch new log entries.
         * It is used within a stream as the sink of characters written 
         * by the user. At th end of ists lifetime it will then dispatch
         * the message if it has content.
         *
         * @relates TREX::utils::log::entry
         * @ingroup utils
         * @author Frederic Py <fpy@mbari.org>
         */
        class entry_sink {
        public:
          /** @brief Stream base char type */
          typedef entry::char_type           char_type;
          /** @brief output stream tag
           *
           * Indicates that this class is used to manage an output stream
           */
          typedef boost::iostreams::sink_tag category;
          
          /** @brief Default constructor
           *
           * Create a non functional new instance.
           *
           * @note this constrauctor exists for the sole reason of better 
           * handling C++ containers. It should be avoided to use it directly.
           */
          entry_sink() {}
          /** @brief Copy constructor
           *
           * @param[in] other Another instance
           *
           * Transfert the entry ownershipe from @p owner to the newly 
           * created instance.
           *
           * @post @p other is non functional
           */
          entry_sink(entry_sink const &other):m_log(other.m_log) {
            m_entry.swap(other.m_entry);
          }
          /** @brief Destructor
           *
           * This class handle the dispatch of the manage entry (if any). 
           * It will dispatch it entry if this entry has content the 
           * text_klog that initially created it
           *
           * @sa TREX::utils::log::entry::has_content()
           * @sa TREX::utils::log::text_log
           */
          ~entry_sink();
          
          /** @brief Add content to the entry
           *
           * @param[in] s A string 
           * @param[in] n Number of characters in @p s
           *
           * Append @p n characters of @p s into the entry. This operation 
           * will be done only if this instance is functional (ie associted to 
           * a text_log and currently owning an entry)
           *
           * @retval n on success
           * @retval 0 if this entry is non functional
           */
          std::streamsize write(char_type const *s, std::streamsize n); 
          
        private:
          entry_sink(boost::shared_ptr<sig_impl> const &dest,
                     entry::id_type const &who,
                     entry::id_type const &what)
          :m_log(dest), m_entry(new entry(who, what)) {}
        
          entry_sink(boost::shared_ptr<sig_impl> const &dest,
                     entry::date_type const &when,
                     entry::id_type const &who, entry::id_type const &what)
          :m_log(dest), m_entry(new entry(when, who, what)) {}
        
          WEAK_PTR<sig_impl> m_log;
          mutable entry::pointer    m_entry;
        
          friend class ::TREX::utils::log::text_log;
        }; // class TREX::utils::log::details::entry_sink
        
        /** @brief Entry construction stream
         *
         * This class presents the basic implementation of a log entry 
         * construction stream. It is based on the entry_sink class and 
         * just adapt it to C++ stream
         * 
         * @relates TREX::utils::log::stream
         * @ingroup utils
         */
        typedef boost::iostreams::stream<entry_sink> stream_impl;
      
      } // TREX::utils::log::details
    } // TREX::utils::log
  } // TREX:utils
} // TREX

#endif // H_trex_utils_log_log_stream
