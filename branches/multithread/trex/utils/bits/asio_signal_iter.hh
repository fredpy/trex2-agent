/** @file "trex/utils/bits/asio_signal_iter.hh"
 *
 * @brief Asynchronous signal manager specializations
 *
 * @author Frederic Py
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
#ifdef H_trex_utils_asio_signal
# ifndef BOOST_PP_IS_ITERATING

# include <boost/preprocessor/repetition.hpp>
# include <boost/preprocessor/iteration/iterate.hpp>
# include "asio_signal_base.hh"

#  ifndef ASIOSIG_MAX_SIZE
#   define ASIOSIG_MAX_SIZE 3 // A maximum of 3 arguments should be more than enough
#  endif // ASIOSIG_MAX_SIZE

// Generate specializations for 0 to ASIOSIG_MAX_SIZE arguments
#  define BOOST_PP_ITERATION_LIMITS (0, ASIOSIG_MAX_SIZE)
#  define BOOST_PP_FILENAME_1       "trex/utils/bits/asio_signal_iter.hh"

#  include BOOST_PP_ITERATE()

#  undef BOOST_PP_ITERATION_LIMITS
#  undef BOOST_PP_FILENAME_1

# else // BOOST_PP_IS_ITERATING

#  define ASIO_SIGNAL_NUM_ARGS BOOST_PP_ITERATION()
#  include "asio_signal_template.hh"

# endif // BOOST_PP_IS_ITERATING
#endif //  H_trex_utils_asio_signal
