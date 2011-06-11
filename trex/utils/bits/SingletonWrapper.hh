/** @file "SingletonWrapper.hh"
 * @brief Defintion of the SingletonWrapper class
 *
 * This header is for internal use and define the
 * SingletonWrapper class that encapsulate a pheonix
 * singleton and control its life-time.
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
#ifndef H_SingletonWrapper
# define H_SingletonWrapper

# include "SingletonDummy.hh"

namespace TREX {
  namespace utils {

    namespace internal {

      /** @brief Factory for SingletonWrapper
       * @tparam Ty type of the singleton
       *
       * This class is a functor that allows the production of
       * SingletonWrapper to create a  new phoenix singleton when
       * required
       *
       * @author Frederic Py <fpy@mbari.org>
       * @relates SingletonWrapper<Ty>
       * @ingroup utils
       */
      template<typename Ty>
      class swrapper_factory;

    } 
    
    /** @brief Phoenix singleton wrapper
     * @tparam Ty type of the singleton
     *
     * This class is used internally to encapsulate and control the
     * life-time of a phoenix singleton accessed through SingletonUse<Ty>
     *
     * If one want to implement the classs @c Sing as a pure singleton --
     * meaning that it cannot be created otherwise -- he needs to define
     * all its constructors (or at least its destructor) as private and than
     * make @c SingletonWrapper<Sing> friend of this class.
     * 
     * @bug There's a potential issue with dynamic library loading on
     * certain platforms. The resolution would be potentially to use @c
     * boost::flyweight with  @c intermodule_holder as in the BasicSymbol
     * class. But as for now it appears to work and I don't know really how
     * to reimplement this class and still have some guarantees that I can
     * somehow guarantee that no intermediate class would be created.
     *
     * @note The instance operator appears to be very slow sometimes. I need
     * to find a way to improve it.
     * 
     * @sa SingletonUse<Ty>
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    template<typename Ty>
    class SingletonWrapper :private internal::SingletonDummy {
    public:
      /** @brief singleton creation/reference attachement
       *
       * @param inst A singleton placeholder
       *
       * This method indicate that a new SingletonUse is referring to
       * the singleton @a Ty. If this singleton does not exists it is
       * created. In any case the referrence counter associated is
       * incremented by 1.
       *
       * @post The singleton @a Ty exist
       * @return A reference to the singleton 
       * @sa void internal::SingletonDummy::attach(std::string const &, internal::sdummy_factory const &)
       */
      static Ty *attach();
      /** @brief singleton refence detachment
       *
       * This method indicate that a SingletonUse is not referring to
       * this singleton @a Ty anymore. It decrements the referrence counter
       * by 1 and, if this counter reaches 0, destroy the singleton @a Ty
       *
       * @sa void internal::SingletonDummy::detach(std::string const &)
       */
      static void detach();

    private:
      /** @brief Constructor */
      SingletonWrapper();
      /** @brief Destructor */
      ~SingletonWrapper();

      /** @brief Singleton identifier
       *
       * used to give an identifier for the singleton. This type is
       * based on @c typeid C++ primitive
       *
       * @return the name associated to @a Ty
       */
      static std::string name();

      /** @brief Singleton
       *
       * This is where the singleton is stored
       */
      Ty m_value;
      
      friend class internal::swrapper_factory<Ty>;
    }; // TREX::utils::SingletonWrapper<>

  }
}

# define In_H_SingletonWrapper
#  include "SingletonWrapper.tcc"
# undef In_H_SingletonWrapper

#endif // H_SingletonWrapper
