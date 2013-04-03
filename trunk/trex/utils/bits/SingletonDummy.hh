/** @file "SingletonDummy.hh"
 * @brief Defintion of the SingletonDummy class
 *
 * This header is for internal use and define an
 * abstract definition of the SingletonWrapper
 * used to create and manipulate phoenix singletons
 * internally
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
#ifndef H_SingletonDummy
# define H_SingletonDummy

# include <string>

# include <boost/utility.hpp>

# include "SingletonServer_fwd.hh"

namespace TREX {
  namespace utils {
    namespace internal {
      
      class SingletonDummy;
      
      /** @brief Abstract factor for singleton
       *
       * This functor provides an abstract functor to produce new
       * singleton wrappers
       *
       * @author Frederic Py
       *
       * @relates SingletonDummy
       * @ingroup utils
       */
      struct sdummy_factory {
	/** @brief Destructor */
	virtual ~sdummy_factory() {};
	/** @brief singelton wrapper creation
	 *
	 * This class is used by SingletonServer to crweates
	 * the wrapper for a given singleton when this one does
	 * not exist yet.
	 *
	 * @return The newly created singleton wrapper
	 */
	virtual SingletonDummy *create() const =0;
      }; // TREX::utils::internal::sdummy_factory

      /** @brief Abstract class for singleton control
       *
       * This class implements the general utilites used to control
       * the lifetime of a phoenix singleton. It is derived by
       * SingeltonWrapper that is the class that really encapsulates
       * the singleton instance. This class just provide an abstract
       * entry point to ease the registering andf life-time control
       * of the singletons.
       *
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup utils
       */
      class SingletonDummy: boost::noncopyable {
      protected:
	/** @brief Constructor */
	SingletonDummy();
	/** @brief Destructor */
	virtual ~SingletonDummy() =0;
	
	/** @brief Singleton creation
	 * @param name singleton identifier
	 * @param factory singleton creation functor
	 *
	 * This method connect to the SingletonServer and looks for
	 * the existance of the wrapper to the key @a name if this one
	 * does not exist, it uses @a factory to create this neew entry
	 * in the server.
	 *
	 * @post A new singleton is attached to the identifier @a name
	 * @sa SingletonDummy *sdummy_factory::create()
	 */
	static SingletonDummy *attach(std::string const &name, 
                                      sdummy_factory const &factory);
	/** @brief reference end notification
 	 * @param name singleton identifier
	 *
	 * This methods notifies that the singleton @a name just lost
	 * one reference. If the count reach 0 this singleton is then
	 * destroyed from the SingletonServer
	 */
	static void detach(std::string const &name);
        static void disable();

      private:
	/** @brief reference counting increment */
	void incr_ref() const;
	/** @brief reference counting decrement */
	bool decr_ref() const;

	/** @brief reference counter */
	mutable size_t ref_counter;

	friend class SingletonServer;
      }; // TREX::utils::internal::SingletonDummy

    }
  }
}

#endif // H_SingletonDummy
