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
#ifndef H_SingletonWrapper
# define H_SingletonWrapper

# include "dummy.hh"

namespace TREX {
  namespace utils {
    namespace singleton {
      namespace internal {

        template<typename Ty>
        struct swrapper_factory;

      } // TREX::utils::singleton::internal
    
      /** @brief singleton instance wrapper
       * 
       * This class wrap a typed singleton instance and handle
       * its creation and destruction along with the connection 
       * to the central singleton server.
       *
       * It is used internallly by the singleton::use class to 
       * access to the singleton and manage its lifetime.
       *
       * @tparam Ty the type of the wrapped singleton
       *
       * @pre @p Ty is default constructible
       * @pre @p The default constructor and destructor of Ty 
       * are accessible by this class
       *
       * @sa TREX::utils::singleton::use
       * @author Frederic Py <fredpy@gmail.com>
       */
      template<typename Ty>
      class wrapper :private internal::dummy {
      public:
        /** @brief New reference to singleton
         *
         * This method notifies that the singleton as a new client.
         * It creates if needed the singleton isntance and increment 
         * its reference counter by 1
         * 
         * @return A pointer to the singleton instance
         * @post the reference counter for this singleton is incremented by 1
         * @sa wrapper::detach
         */
        static Ty *attach();
        /** @brief Dereference singleton
         *
         * Notifies that one client of this singleton no longer refers to it.
         * As a result it decrease the singleon reference counter and, if the
         * counter reaches 0 destroy the singleton instance 
         *
         * @pre The reference counter is greater than 0
         * @sa wrapper::attach
         */
        static void detach();
            
      private:
        /** @brief Constructor
         *
         * Create  a new instance and initialize the singleton 
         * instance of Ty
         */
        wrapper();
        /** @brief Destructor
         */
        ~wrapper();

        /** @brief Singleton identifier
         *
         * The identifier used to differentiate this singleton type 
         * from others. Its current implementation is based on C++ 
         * @c typeid which gives the mangled name of the type.
         *
         * @return A unique identifier for @p Ty
         */
        static std::string name();
        
        /** @brief The singleton instance
         *
         * This is were the singleton instance is stored.
         */
        Ty m_value;
      
        friend struct internal::swrapper_factory<Ty>;
      }; // TREX::utils::singleton::wrapper<>
    }
  }
}

# define In_H_SingletonWrapper
#  include "wrapper.tcc"
# undef In_H_SingletonWrapper

#endif // H_SingletonWrapper
