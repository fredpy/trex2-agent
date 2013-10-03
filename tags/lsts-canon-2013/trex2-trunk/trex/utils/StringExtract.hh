/* -*- C++ -*-
 * $Id$
 */
/** @file "StringExtract.hh"
 * @brief Utilities for values extraction from string
 *
 * @note This header defines classes that should exist in the
 * boost Conversion library. It will probably be done in the
 * future.
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
#ifndef _STRINGEXTRACT_HH
# define _STRINGEXTRACT_HH

# include <sstream>
# include <typeinfo>

# include <boost/algorithm/string/case_conv.hpp>
# include <boost/date_time/posix_time/posix_time.hpp>
# include <boost/date_time/local_time/local_time.hpp>

namespace TREX {
  namespace utils {
 
    /** @brief Bad string cast exception.
     *
     * This exception is used when a string_cast has failed.
     * This may occur on parse error for the given type.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    class bad_string_cast :public std::bad_cast {
    public:
      /** @brief Constructor
       *
       * @param message A descriptive message for the exception.
       */
      bad_string_cast(std::string const &message) throw()
	:_message(message) {}
      /** @brief Destructor */
      ~bad_string_cast() throw() {}
      
      /** @brief Exception message.
       *
       * @return The message describing the exceptional event.
       */
      char const *what() const throw() {
	return _message.c_str();
      }
    private:
      /** @brief Exception message */ 
      std::string _message; 
    }; // bad_string_cast

    /** Checked char * to std::string conversion
     *
     * @param str A C string
     *
     * This convert a C string to a std::string checking if @c str
     * is not @c NULL. Indeed std::string does not check it and
     * @c std::string(NULL) has an undefined behavior. 
     *
     * @return A std::string corresponding to the value of @e str.
     * If @e str is @c NULL, it returns an empty std::string.
     * @ingroup utils
     */
    inline std::string checked_string(char const *str) {
      if( NULL==str )
	return std::string();
      else
	return str;
    }


    /** @brief Convert string to type.
     *
     * @tparam Ty      Destination type
     * @param in      A string
     * @param format  A format descriptor
     *
     * This method extract a value of type @e Ty from @e in.
     *
     * @return The extracted value.
     *
     * @throw bad_string_cast Error while trying to parse @e in
     * @ingroup utils
     */
    template<typename Ty>
    Ty string_cast(std::string const &in, 
		   std::ios_base &(*format)(std::ios_base &) = std::dec) {
      Ty result;
      std::istringstream iss(in);
      
      if( (iss>>format>>result).fail() ) {
	std::ostringstream message;
	
	message<<"string_cast : unable to parse \""<<in<<"\"";
	throw bad_string_cast(message.str());
      }
      return result;
    }

    /** @brief string casting specialization
     * @param in A string
     * @param format A format
     *
     * @return A copy of @p in
     * @ingroup utils
     */ 
    template<>
    inline std::string string_cast<std::string>(std::string const &in, 
						std::ios_base &(*format)(std::ios_base &)) {
      return in;
    }
    
    template<>
    inline bool string_cast<bool>(std::string const &in,
                           std::ios_base &(*format)(std::ios_base &)) {
      
      bool result;
      std::istringstream iss(in);
      
      if( (iss>>format>>std::boolalpha>>result).fail() )
        return 0!=string_cast<int>(in, format);
      return result;
    }
    
    template<>
    inline boost::posix_time::ptime string_cast<boost::posix_time::ptime>(std::string const &in,
                                                                          std::ios_base &(*format)(std::ios_base &)) {

      typedef boost::posix_time::time_input_facet facet;
      std::istringstream iss(in);
      
      iss.imbue(std::locale(iss.getloc(), new facet("%Y-%m-%dT%H:%M:%S%F")));
      boost::posix_time::ptime date(boost::date_time::not_a_date_time);
      
      if( (iss>>date).fail() ) {
        std::ostringstream message;
	
	message<<"string_cast : unable to parse \""<<in<<"\"";
	throw bad_string_cast(message.str());
      } else {
        std::string tz(iss.str().substr(iss.tellg()));
        if( "Z"!=tz && !tz.empty() ) {
          try {
            date -= boost::posix_time::duration_from_string(tz);
          } catch(boost::bad_lexical_cast const &e) {
            throw bad_string_cast(e.what());
          }
        }
      }
      return date;
    }
  
    /** @brief Convert string to type.
     *
     * @tparam Ty       Destination type
     * @param on_error A value for erro handling
     * @param in       A string
     * @param format   A format descriptor
     *
     * This method extract a value of type @e Ty from @e in. If the parsing fails it
     * will return @e on_error. 
     *
     * @return The extracted value or @e on_error if parsing of @e in failed.
     *
     * @sa Ty string_cast(std::string const &, std::ios_base &(*)(std::ios_base &))
     * @ingroup utils
     */
    template<typename Ty>
    Ty string_cast(Ty const &on_error, std::string const &in, 
		   std::ios_base &(*format)(std::ios_base &) = std::dec) {
      try {
	return string_cast<Ty>(in, format);
      } catch(bad_string_cast &e) {
	return on_error;
      }
    }

  } // TREX::utils
} // TREX

#endif // _STRINGEXTRACT_HH
