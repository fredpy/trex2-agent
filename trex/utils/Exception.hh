/** @file "trex/utils/Exception.hh"
 * @brief TREX exception base
 *
 * This file declares the Exception class used a basis for all the exceptions 
 * related to TREX components.
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
