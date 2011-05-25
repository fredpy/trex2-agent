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
#ifndef _STRINGEXTRACT_HH
# define _STRINGEXTRACT_HH

# include <sstream>
# include <typeinfo>

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
     * @return A copy of @a in
     * @ingroup utils
     */ 
    template<>
    inline std::string string_cast<std::string>(std::string const &in, 
						std::ios_base &(*format)(std::ios_base &)) {
      return in;
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
