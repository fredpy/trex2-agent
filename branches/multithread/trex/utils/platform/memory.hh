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
/** @file "trex/utils/platform/memory.hh"
 * @brief C++11 Unique pointer adapter header
 *
 * This header provide macros that uses either std::auto_ptr or
 * its C++11 counterpart std::unique_ptr when trex was compiled with 
 * C++11 support
 *
 * It provide two macros that allow to handle unique pointers as if 
 * their implementation was @e relativelly similar to C++11 specs. 
 * Specifically it provides a macro to get the preferred type for 
 * unique pointers and a macro to move the pointer from one instance 
 * to another
 *
 * @note It is recommended to replace all references to the standard 
 *   memory header by this header
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */
#ifndef H_trex_memory
# define H_trex_memory

# include <memory>
# include "bits/cpp11.hh"

# ifdef DOXYGEN
/** @brief Unique pointer type
 *
 * The type to be used for unqie pointers. It is either std::auto_ptr 
 * or std::unique_ptr depending on C++11 support
 */
#  define UNIQ_PTR computed_type
/** @brief Unique pointer moving function
 * 
 * @param[in] ptr A @c UNIQ_PTR instance
 *
 * The "function" to call in order to move @p ptr to another instance. 
 * Moving a unique pointer means that its ownership is transferred to
 * another instance.
 *
 * Before C++11 this operation was done implicitelly which has been 
 * corrected with the introduction of std::unique_ptr meant to 
 * replace the dperecated std::auto_ptr
 */
#  define STD_MOVE(ptr)

#  define SHARED_NS  computed_namespace

# else

#  ifdef CPP11_HAS_UNIQUE_PTR
#   define UNIQ_PTR  std::unique_ptr
#   define STD_MOVE(ptr) std::move(ptr)
#   define MOVE_ARG(type) type &&
#  else // CPP11_HAS_UNIQUE_PTR
#   define UNIQ_PTR  std::auto_ptr
#   define STD_MOVE(ptr) ptr
#   define MOVE_ARG(type) type
#  endif // CPP11_HAS_UNIQUE_PTR

#  ifdef CPP11_HAS_SHARED_PTR
#   define SHARED_NS std
#  else // CPP11_HAS_SHARED_PTR
#   include <boost/make_shared.hpp>
#   include <boost/weak_ptr.hpp>
#   include <boost/enable_shared_from_this.hpp>

#   define SHARED_NS boost
#  endif // CPP11_HAS_SHARED_PTR

# endif // DOXYGEN

# define SHARED_PTR  SHARED_NS::shared_ptr
# define WEAK_PTR    SHARED_NS::weak_ptr
# define MAKE_SHARED SHARED_NS::make_shared
# define ENABLE_SHARED_FROM_THIS SHARED_NS::enable_shared_from_this

#endif // H_trex_memory
