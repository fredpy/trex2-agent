/* -*- C++ -*- */
/** @file "trex/utils/log/entry.hh"
 * @brief Definition of a log entry
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
#ifndef H_trex_utils_log_entry
# define H_trex_utils_log_entry

# include "log_fwd.hh"
# include <boost/optional.hpp>

namespace TREX {
  namespace utils {
    namespace log { 

      /** @brief A log entry
       *
       * This class is used to stroe and transfert log emssages entries. 
       * It provides :
       * @li An optional date expressed as an integer
       * @li A source which describes the generator of this entry
       * @li A kind giving the type of the message (including, but not
       *     limited to @c log::info, @c log::warn, @c log::error)
       * @li The message content itself
       *
       * Every time A new log message is built using a @c log::stream it is
       * dispatched on completion as an entry which can be then capture by 
       * log handlers
       *
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup utils
       */
      class entry {
      public:
        
# ifdef DOXYGEN	// doxygen tweaks
	/** Type for a date
	 *
	 * The type used to represent a date. While it is often an integer type 
	 * it is better to just consider it as a type that is ordered and can be 
	 * written in a regular output stream
	 */
	typedef unspecified date_type;
# else  // DOXYGEN
        typedef unsigned long long date_type;
# endif // DOXYGEN
        
	/** @brief identifiers' type
	 *
	 * The type used for both that source and kind of the entry.
	 */
        typedef ::TREX::utils::log::id_type id_type;
	/** @brief Message content type
	 *
	 * The type used to store the message content.
	 */
        typedef std::string                 msg_type;
        
	/** @brief Pointer type
	 *
	 * The type used to refer to an entry. This typ[e is the 
	 * expected argument of log message handlers.
	 */
        typedef boost::shared_ptr<entry> pointer;
        
	/** @brief Desturctor */
        ~entry() {}
        
	/** @brief Check for date
	 *
	 * Checks if this entry has a date field
	 *
	 * @retval true if a date is associated to this message
	 * @retval false otherwise
	 * @sa date()
	 */
        bool is_dated() const {
          return m_date;
        }
	/** @brief Entry date
	 *
	 * @pre @c is_dated()
	 *
	 * @return The date fassociated to this entry
	 * @sa is_dated()
	 */
        date_type const &date() const {
          return *m_date;
        }
	/** @brief Entry source 
	 *
	 * @return A symbol describing the source of the message
	 */
        id_type const &source() const {
          return m_source;
        }
	/** @brief Message type
	 *
	 * @return A symbol describing the type of the message
	 */
        id_type const &kind() const {
          return m_kind;
        }
	/** @brief Check for content
	 *
	 * @retval true if the entry have a non emepty content
	 * @retval false otherwise
	 *
	 * @note this method is mostly for internally use to help decide if 
	 *       a newly created entry is worth sending. It checks that the 
	 *       creator did wirte at least few characters before closing
	 *       the stream. While A content could still be empty, this means 
	 *       that user did force the send of this message by writing a new line.
	 * @sa content()
	 */
        bool has_content() const {
          return m_pending_nl || !m_content.empty();
        }
	/** @brief Message content
	 *
	 * @return The message content
	 *
	 * @note Our system removes automatically from the content the last character 
	 *       if it was a new line. This allows to simplify message display while 
	 *       avoiding empty lines in the log file (unless explicitely requrested 
	 *       by 2 successive new lines)
	 */
        msg_type const &content() const {
          return m_content;
        }
        
      private:
        typedef msg_type::value_type char_type;
        typedef msg_type::size_type  size_type;
        
        entry(id_type const &src, id_type const &kind)
        :m_source(src), m_kind(kind), m_pending_nl(false) {}
        
        entry(date_type const &date, id_type const &src, id_type const &kind)
        :m_date(date), m_source(src), m_kind(kind), m_pending_nl(false) {}
        
        size_type write(char_type const *s, size_type n);
        
        
        boost::optional<date_type> const m_date;
        id_type const m_source;
        id_type const m_kind;
        msg_type m_content;
        bool m_pending_nl;
                
        friend class TREX::utils::log::details::entry_sink;
        friend class log_pipe;
      }; // class TREX::utils::log::entry
            
    } // TREX::utils::log
  } // TREX::utils
} // TREX

#endif // H_trex_utils_log_entry
