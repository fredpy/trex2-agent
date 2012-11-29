/** @file "trex/utils/Hashable.hh"
 * @brief TR1 hashing utilities
 *
 * This file provides some utilities to ease the
 * definition of the hash function for a class.
 *
 * @note This implementation is currently based on boost
 * TR1 compatibility hash definition
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
#ifndef H_Hashable
# define H_Hashable

# include <cstddef>
# include <boost/functional/hash.hpp>

namespace TREX {
  namespace utils {
    class Hashable;
    
    /** @brief Hashing proxy for boost::hash
     *
     * @param x Insance to hash
     *
     * This method is used internally by boost::hash functor to
     * compute the hash value
     *
     * @return The hash value of @a x
     *
     * @sa std::size_t Hashable::hash() const
     * @relates TREX::utils::Hashable
     * @ingroup utils
     */
    size_t hash_value(TREX::utils::Hashable const &x);
    
    
    /** @brief Abstract class that supports hash
     *
     * This class provides an easy way to support hash function
     * (and consequently TR1 collections) for classes.
     *
     * To do so one just need to inherit from this class and
     * implements the hash() method accordingly
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    class Hashable {
    public:
      /** @brief Constructor */
      Hashable() {}
      /** @brief Destructor */
      virtual ~Hashable() {}
      
    protected:
      /** @brief hashing method
       *
       * This method is called by the TR1` hash functor to get
       * the hash value of current instance.
       *
       * @return computed hash value
       */
      virtual size_t hash() const =0;
      
      friend std::size_t hash_value(TREX::utils::Hashable const &x);
    }; // TREX::utils::Hashable
    
    
    inline std::size_t hash_value(TREX::utils::Hashable const &x) {
      return x.hash();
    }
    
  } // boost::utils
} // boost

#endif // H_Hashable
