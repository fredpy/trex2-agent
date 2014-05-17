/** @file "SingletonServer.hh"
 * @brief Defintion of the SingletonServer class
 *
 * This header is for internal use and define the
 * SingletonServer class. This class is the centralized
 * place where all the singletons are maintained
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
#ifndef H_SingletonServer
# define H_SingletonServer

# include "../bits/dummy.hh"

# include <boost/thread/recursive_mutex.hpp>

# include <map>

namespace TREX {
  namespace utils {
    namespace singleton {
      namespace internal {
      
        /** @brief Singletons server
         *
         * This class manages the life times of all the singleton
         * manipulated through TREX::utils::singleton::use. It is 
         * in fact the only true singleton and its lifetime is 
         * guaranted from its first access to the end of the program.
         *
         * @note All operations from this class are thread safe.
         *
         * @author Frederic Py
         */
        class server :boost::noncopyable {
        public:
          /** @brief Access server
           *
           * Give access ot the unique instance of server and create it 
           * on its first call.
           *
           * @return the server unique instance 
           */
          static server &instance() throw();
        
          /** @brief Attach new singleton reference 
           *
           * @param[in] id A unique identifier
           * @param[in] factory A method to creat the instance 
           *
           * Give access and increment the reference counter to the 
           * singleton named @p id and create it with @p factory if 
           * @p id do not exist.
           *
           * @return A pointer to the singleton @p id
           *
           * @sa server::detach(std::string const &)
           */
          dummy *attach(std::string const &id,
                        sdummy_factory const &factory);
          /** @brief Detach singleton reference
           *
           * @param[in] id A unique identifier
           *
           * Notifies that the singleton associated to @p id lost a 
           * reference. If this was the last reference then the singleton 
           * @p id is destroyed.
           *
           * @retval true if @p id has been destroyed
           * @retval false if @p id still have more references
           */
         bool detach(std::string const &id);
        
        private:
         
          server() throw();
          ~server() throw();
          
          typedef boost::recursive_mutex mutex_type;
          typedef mutex_type::scoped_lock lock_type;
          
          typedef std::map<std::string, dummy *> single_map;
        
          mutex_type m_mtx;
          single_map m_singletons;
        
          static void make_instance() throw();
          static server *s_instance;
        };
        
      }
    }
  }
}

#endif // H_SingletonServer
