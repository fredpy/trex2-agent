/* -*- C++ -*- */
/** @file "utils/base/Exception.hh"
 * @brief TREX exception base
 *
 * This file declares the Exception class used a basis for all the exceptions 
 * related to TREX components.
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */   
#ifndef H_Exception
# define H_Exception

# include <stdexcept>
# include "IOstreamable.hh"

namespace TREX {
  namespace utils {
    
    /** @brief TREX exception
     *
     * This class provides an abstract definition for TREX related exception.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    class Exception :public std::runtime_error, public ostreamable {
    public:
      /** @brief Constructor
       * @param[in] msg A text message
       *
       * Create a new instance with associated message @p msg
       */
      Exception(std::string const &msg) throw();
      /** @brief Destructor */
      virtual ~Exception() throw();
			
    protected:
      std::ostream &print_to(std::ostream &out) const;
    }; // class TREX::utils::Exception
    
  } // TREX::utils
}  // TREX

#endif // H_Exception
