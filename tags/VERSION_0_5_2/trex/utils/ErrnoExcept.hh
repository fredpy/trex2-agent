/** @file "trex/utils/ErrnoExcept.hh"
 *
 * @brief ErrnoExcept definition.
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
#ifndef H_ErrnoExcept
# define H_ErrnoExcept

# include <cerrno>

# include "Exception.hh"

namespace TREX {
  /** @brief TREX basic utilities
   *
   * This namespace embeds different classes that are not necessarily
   * specific to TREX but eases different implementation aspect of it.
   *
   * You'll find here classes providing very basic features such as
   * thread management, exceptions, and other general design patterns.
   *
   * Normally classes and code defined here is provided by the dynamic
   * library named TREXutils.
   *
   * @ingroup utils
   */
  namespace utils {
		
    /** @brief @c errno handling exception
     *
     * This exception class can be used when a C error that modifies the
     * value of @c errno is detected. It embeds both the value of @c errno
     * and the associated message as extracted by @c strerror C primitive.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    class ErrnoExcept :public Exception {
    public:
      /** @brief Constructor
       * @param[in] from Source of the error
       *
       * Creates a new instance with the indication that the error came from 
       * @p from. Extracts automatically the value of @c errno and its 
       * associated message to build the exception message 
       * "@p from: errno message"
       */
      explicit ErrnoExcept(std::string const &from) throw()
	:Exception(build_message(from)), _errno(errno) {}
      /** @brief Constructor
       *
       * @param[in] from Source of the error
       * @param[in] what The message of associated 
       *
       * Creates a new instance with the indication that the
       * error came from @p from and the reason associated is
       * @p what. 
       *
       * @note This constructor does not collect the @c errno
       * value nor its associated message. Instead it considers
       * that @p what is the correct message. This somehow
       * defies the purpose of this exception and may change
       * in the future.
       */
      ErrnoExcept(std::string const &from, std::string const &what) throw()
	:Exception(build_message(from, what)), _errno(0) {}
      /** @brief Destructor */
      virtual ~ErrnoExcept() throw() {}
      
      /** @brief Gets associated @c errno value
       *
       * @return the value of @c errno when this exception was thrown
       */
      int get_errno() const {
	return _errno;
      }
      
    private:
      /** @brief Saved value of @c errno
       *
       * This attribute is used to storethe @c errno valuee collected during the 
       * exception construction. 
       */
      int _errno;
      
      /** @brief Exception message construction.
       * 
       * @param[in] from source of the exception
       * @param[in] what message content
       * 
       * This method is used internally to create the message during
       * exception construction. 
       * 
       * @note when the @p what argument is not provided the calls collect 
       * directly the message constent using @c strerror standard function 
       * 
       * @{
       */
      static std::string build_message(std::string const &from, 
				       std::string const &what) throw();
      
      static std::string build_message(std::string const &from) throw();
      /** @} */
    }; // class TREX::utils::ErrnoExcept
		
  } // TREX::utils     
} // TREX

#endif // H_ErrnoExcept
