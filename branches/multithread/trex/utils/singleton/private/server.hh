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

# include "../bits/dummy.hh"

namespace TREX {
  namespace utils {
    namespace singleton {
      
      class server :boost::noncopyable {
      public:
        static server &instance();
        
        dummy *attach(std::string const &id,
                      details::dummy_factory const &factory);
        bool detach(std::string const &id);
        
      private:
        server();
        ~server();
        
        typedef boost::recursive_mutex  mutex_type;
        typedef mutex_type::scoped_lock lock_type;
        
        typedef std::map<std::string, dummy *> single_map;
        
        mutex_type m_mtx;
        single_map m_singletons;
        
        
        static void make_instance();
        static server *s_instance;
      }; // TREX::utils::singleton::server
      
    } // TREX::utils::singleton
  } // TREX::utils
} // TREX

#endif // H_SingletonServer
