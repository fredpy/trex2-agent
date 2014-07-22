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
/** @file "trex/config/cpp11_deleted.hh"
 *
 * @brief Support of C++11 "=delete" keyword.
 *
 * This header provide macro to support the "=delete" keyword on methods 
 * when compiled with C++11 standard cpompiler.
 *
 * The "=delete" method modifier allow to specify that a method of a class
 * has no implementation and inform the compiler to not try to create 
 * provide a default implementation for it.
 *
 * This is especially usefull when we ndo not want the compiler to generate 
 * a default or copy contstructor for this class.
 */
#ifndef H_trex_config_cpp11_deleted
# define H_trex_config_cpp11_deleted

# include "bits/cpp11.hh"

# ifdef DOXYGEN

/** @brief mfunction deletion macro
 *
 * This macro is used to define how the compiler handle function deletion. 
 * If the compiler support C++11 it will be replaced by 
 * @code
 *  =delete
 * @endcode
 * for older compiler it will just silently ignore it.
 *
 * For example if one person want to have a class foo with no default 
 * constructor it can write it as follow:
 * @code 
 * #include <trex/utils/platform/cpp11_deleted.hh>
 *
 * class foo {
 * public:
 *   foo(int val);
 *
 * private:
 *   foo() DELETED;
 * };
 * @endcode
 * And not implement the foo() constructor. On a C++11 compiler DELETED will 
 * indicate explicitely that this constructor has no code which will allow 
 * earlier detection (at compile time vs link time) of unauthorized call of
 * this constructor.
 *
 * @author Frederic Py
 * @ingroup utils
 */
# define DELETED platform-dependent

# elseif defined(CPP11_HAS_DELETED_FUNCTIONS)

# define DELETED =delete

# else // CPP11_HAS_DELETED_FUNCTIONS

# define DELETED

# endif // CPP11_HAS_DELETED_FUNCTIONS

#endif  // H_trex_config_cpp11_deleted

