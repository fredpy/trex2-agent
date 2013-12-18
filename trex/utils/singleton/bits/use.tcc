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
#ifndef In_H_trex_utils_singleton_use
# error "Cannot include tcc files outside of their corresponding header"
#else

#include "wrapper.hh"

namespace TREX {
  namespace utils {
    namespace singleton {
      
      /*
       * class TREX::utils::singleton::use<>
       */
      template<typename Ty>
      void use<Ty>::disable() {
        wrapper<Ty>::disable_server();
      }
    
    
      template<typename Ty>
      use<Ty>::use():m_instance(wrapper<Ty>::attach()) {}

      template<typename Ty>
      use<Ty>::use(use<Ty> const &):m_instance(wrapper<Ty>::attach()) {}

      template<typename Ty>
      use<Ty>::~use() {
        m_instance = 0x0;
        wrapper<Ty>::detach();
      }

      template<typename Ty>
      Ty &use<Ty>::instance() const {
        return *m_instance;
      }

      template<typename Ty>
      Ty &use<Ty>::operator*() const {
        return instance();
      }

      template<typename Ty>
      Ty *use<Ty>::operator->() const {
        return &instance();
      }

    }
  }
}

#endif 
