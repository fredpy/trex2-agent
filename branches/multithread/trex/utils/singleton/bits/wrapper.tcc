/* -*- C++ -*- */
/** @file "SingletonWrapper.tcc"
 * @brief SingletonWrapper implementation
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
#ifndef In_H_SingletonWrapper
# error "Cannot include tcc file outside of its corresponding header"
#else

#include <typeinfo>

namespace TREX {
  namespace utils {
    namespace singleton {
      namespace details {
        
        template<typename Ty>
        class wrapper_factory :public dummy_factory {
          dummy *create() const {
            return new wrapper<Ty>;
          }
        }; // TREX:utils::singleton::details::warrper_factory<>
        
      } // TREX:utils::singleton::details
      
      /*
       * class TREX::utils::singleton::wrapper<>
       */
      
      // statics
      
      template<typename Ty>
      Ty *wrapper<Ty>::attach() {
        wrapper<Ty> *me = static_cast<wrapper<Ty> *>(dummy::attach(name(),
                                                                   details::wrapper_factory<Ty>()));
        return &(me->m_value);
      }
      
      template<typename Ty>
      void wrapper<Ty>::detach() {
        return dummy::detach(name());
      }

      template<typename Ty>
      std::string wrapper<Ty>::name() {
        // Use RTTI for generating a id for this type
        return typeid(Ty).name();
      }
      
      template<typename Ty>
      void wrapper<Ty>::disable_server() {
        dummy::disable();
      }
      
      // structors
      template<typename Ty>
      wrapper<Ty>::wrapper() {}
      
      template<typename Ty>      
      wrapper<Ty>::~wrapper() {}
      
    } // TREX::utils::singleton
  } // TREX::utils
} // TREX

#endif
