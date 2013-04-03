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

# include <boost/thread/recursive_mutex.hpp>

# include <map>

# include "../bits/SingletonDummy.hh"

namespace TREX {
  namespace utils {
    namespace internal {
      
      /** @brief General trex singletons manager
       *
       * This class implements the only pure singleton in T-REX. 
       * Its role is to manage and maintiain the lifetime of all the 
       * objects accessed through SingletonUse. 
       *
       * while SingletonUse provide do not guarantee that the class 
       * referred to as only one single instance (it only ensures that
       * all the SingletonUse access to the same instance) and do not 
       * ensure that the singleton will not be detroyed and recreated 
       * within the program execution. This class on the other hand is 
       * always unique and guaranteed to be the same from its creation 
       * to the end of the program. 
       *
       * Access to the managed "SingletonUse" real instances is protected 
       * here by a mutex which should ensure that their reference counting
       * is properly handled in a multithread environment.
       *
       * @note In order to avoid destruction order issues this implementation
       * do not destroy the initally created instance. While this generate 
       * a small memory leak (at the end of the program we usueally have only 
       * an empty map wialong with a mutex as managed SingletonUse did 
       * destroy themselve) it appeared to be the only safe approach to ensure
       * that this instance won't be destoyed prematurelly.
       */
      class SingletonServer :boost::noncopyable {
      public:
        /**
         * @brief Singleton access
         *
         * Gives access to the singleton instance of this class. 
         * This methofd will also create the instance when first 
         * called in a thread safe way.
         *
         * @return the singleton instance
         */
        static SingletonServer &instance();
        
        
        /** @brief Request singletonUse referred instance
         *
         * @param[in] id An identifier (usually the typeid of the class)
         * @param[in] factory A factory for the requested type
         *
         * Request access to the "singleton" associated to @p id. If no
         * such instance currently exists. This will be created using 
         * @p factory for creation.
         *
         * Additionally this method does increase the reference counter 
         * for this @p id
         *
         * @note This operation is mutex protected to ensure reentrance.
         *
         * @return the wrapper associated to @p id
         *
         * @sa detach(std::string const &)
         */
        SingletonDummy *attach(std::string const &id,
                               sdummy_factory const &factory);
        /** @brief release SingletonUse instance
         *
         * @param[in] id an identifer
         *
         * Notifies that the "singleton" associated to @p id is not 
         * referred anymore by one SingleonUse. As a result its 
         * referrence counter is decreased and iff it reaches 0 the 
         * associated singleton is then destroyed 
         *
         * @note This operation is mutex protected to ensure reentrance.
         * @note This operation is robust to calls with invalid ids. If 
         *     the @p id does not exists it just silently return false
         *
         * @retval true the associeted singleton hass been destroyed
         * @retval false otehrwise
         */
        bool detach(std::string const &id);
        
      private:
        SingletonServer();
        ~SingletonServer();
        
 	typedef boost::recursive_mutex mutex_type;
	typedef mutex_type::scoped_lock lock_type;
               
        typedef std::map<std::string, SingletonDummy *> single_map;
        
        mutex_type m_mtx;
        single_map m_singletons;
        
        static void make_instance();
        static SingletonServer *s_instance;
      };
      
    }
  }
}

#endif // H_SingletonServer
