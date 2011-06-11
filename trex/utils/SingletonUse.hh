/* -*- C++ -*-
 * $Id$
 */
/** @file "SingletonUse.hh"
 * @brief Define the access class to a phoenix singleton
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
#ifndef H_SingletonUse
# define H_SingletonUse

namespace TREX {
  namespace utils {

    /** @brief Singleton accessor
     * @tparam Ty type of the singleton
     *
     * This class is used to access to a phoenix singleton.
     * It also manages the singleton creation and lifetime
     * through reference counting
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    template<typename Ty>
    struct SingletonUse {
    public:
      /** @brief Constructor
       *
       * Create a new instance referring to the singleton @a Ty. If
       * this singleton does not already exist it is created using
       * the default constructor of @a Ty. On any case the reference
       * counter for the singleton @a Ty is incremented by 1.
       */
      SingletonUse();
      /** @overload SingletonUse() */
      SingletonUse(SingletonUse<Ty> const &other);
      /** @brief Destructor
       * Send a notification to the singleton mmanager that one
       * instance less is accessing to the singleton @a Ty if no
       * more instance are refering to it them this singleton will
       * be destroyed
       */
      ~SingletonUse();

      /** @brief Singleton instance
       * @return a reference to the singleton @a Ty
       * @{
       */
      Ty &instance() const;
      Ty &operator*() const;
      Ty *operator->() const;
      /** @} */
    private:

      Ty *m_instance; //!< Singleton_reference
    }; // TREX::utils::SingletonUse<>
  }
}    
# define In_H_SingletonUse
#  include "bits/SingletonUse.tcc"
# undef  In_H_SingletonUse

#endif // H_SingletonUse
