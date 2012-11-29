/** @file "trex/utils/IOstreamable.hh"
 * @brief Standard streams support utilities
 *
 * This file provides some utilities to ease the
 * support of standard stream from user defined classes
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
#ifndef H_IOstreamable
# define H_IOstreamable

# include <iostream>

namespace TREX {
  namespace utils {
    
    class ostreamable;
    class istreamable;
    
  } // TREX::utils
} // TREX

namespace std {
  
  /** @brief Output operator overload
   *
   * @param[in,out] out An output stream
   * @param[in] x An ostreamable instance
   *
   * Writes the value of @p x into @p out
   *
   * @return @p out after the operation
   *
   * @sa TREX::utils::ostreamable::print_to(std::ostream &) const
   */
  ostream &operator<<(ostream &out, TREX::utils::ostreamable const &x);
  /** @brief Input operator overload
   *
   * @param[in,out] in An input stream
   * @param[out] x An istreamable instance
   *
   * Reads the value of @p x from @p in
   *
   * @return @p in after the operation
   *
   * @sa TREX::utils::istreamable::read_from(std::istream &)
   */
  istream &operator>>(istream &in, TREX::utils::istreamable &x);
  
} // std

namespace TREX {
  namespace utils {
    
    /** @brief standard output stream support class
     *
     * This class defines a generic interface to C++ standard
     * output streams.
     *
     * To support the standard << operator one just need to inherit
     * from this class and implements the @c print_to method.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    class ostreamable {
    public:
      /** @brief Constructor. */
      ostreamable() {}
      /** @brief Destructor. */
      virtual ~ostreamable() {}
      
    protected:
      /** @brief output method
       *
       * @param[in,out] out An output stream
       *
       * This method is internally called by the standard output
       * stream operator to write the value of current instance into @p out
       *
       * @sa std::ostream &operator<<(std::ostream &, ostreamable const &)
       */
      virtual std::ostream &print_to(std::ostream &out) const =0;
      
      friend std::ostream &std::operator<<(std::ostream &out,
                                           TREX::utils::ostreamable const &x);
    }; // TREX::utils::ostreamable
    
    /** @brief standard input stream support class
     *
     * This class defines a generic interface to C++ standard
     * input streams.
     *
     * To support the standard >> operator one just need to inherit
     * from this class and implements the @c read_from method.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    class istreamable {
    public:
      /** @brief Constructor */
      istreamable() {}
      /** @brief Destructor */
      virtual ~istreamable() {}
      
    protected:
      /** @brief Input method
       *
       * @param[in] in An input stream
       *
       * This method is internally called by the standard input
       * stream operator to read anew value of this intance from @p in
       *
       * @sa std::istream &std::operator<<(std::istream &, istreamable &)
       */
      virtual std::istream &read_from(std::istream &in) =0;
      
      friend std::istream &std::operator>>(std::istream &in, istreamable &x);
    };
  }
}

namespace std {
  inline ostream &operator<<(ostream &out,
			     TREX::utils::ostreamable const &x) {
    return x.print_to(out);
  }
  
  inline istream &operator>>(istream &in,
			     TREX::utils::istreamable &x) {
    return x.read_from(in);
  }
  
} // std

#endif // H_IOstreamable
