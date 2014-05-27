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
/** @file "trex/utils/platform/chrono.hh"
 * @brief chrono class integration helper header
 *
 * This header allows to include either @chrono or its C++11 
 * std::chrono counterpart depending on the initial compilation 
 * options of TREX
 *
 * This allow to either use chrono as implemented in boost 1.47 and 
 * beyond or the one provided by the C++11 standard if the boost 
 * version is older and the compiler supports C++11
 *
 * @ingroup utils 
 * @author Frederic Py <fpy@mbari.org>
 */
#ifndef H_trex_utils_platform_chrono
# define H_trex_utils_platform_chrono

# include "bits/cpp11.hh"

# ifdef DOXYGEN

/** @brief chrono class namespace 
 *
 * This macro defines the namespace where the chrono template class 
 * is defined. This namespace is either @c boost or @c std depending 
 * on whether c++11 support is active or not
 */
#  define CHRONO_NS platform-dependent

# else // !DOXYGEN
#  ifdef CPP11_HAS_CHRONO
/*
 * Use the standard C++11 chrono header
 */
#   include <chrono>
#   define CHRONO_NS ::std
#  else // !CPP11_HAS_CHRONO
/*
 * Use boost chrono as a replacement
 */
#   include <boost/chrono.hpp>
#   define CHRONO_NS ::boost
#  endif // CPP11_HAS_CHRONO
# endif // DOXYGEN

/** @brief chrono class name
 *
 * The full name of the chrono class with its associated
 * namespace
 */
# define CHRONO CHRONO_NS::chrono

#endif // H_trex_utils_platform_chrono
