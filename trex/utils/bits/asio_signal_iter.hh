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

#  define ASIOSIG_n BOOST_PP_ITERATION()

#  define ASIOSIG_fn_parm(J,I,D) BOOST_PP_CAT(A,I) BOOST_PP_CAT(arg,I)
#  define ASIOSIG_fn_parms BOOST_PP_ENUM(ASIOSIG_n,ASIOSIG_fn_parm,BOOST_PP_EMPTY)
#  define ASIOSIG_fn_args BOOST_PP_ENUM_PARAMS(ASIOSIG_n,arg)

#  if ASIOSIG_n==0
#   define ASIOSIG_typename
#   define ASIOSIG_fn_comma
#  else // ASIOSIG_n>0
#   define ASIOSIG_typename typename
#   define ASIOSIG_fn_comma ,
#  endif // ASIOSIG_n

template<BOOST_PP_ENUM_PARAMS(ASIOSIG_n,class A)>
class asio_signal<void (BOOST_PP_ENUM_PARAMS(ASIOSIG_n,A))>
#  if ASIOSIG_n==1
  :public std::unary_function<A0, void>
#  elif ASIOSIG_n==2
  :public std::binary_function<A0, A1, void>
#  endif // ASIOSIG_n
{
  typedef boost::signals2::signal<void (BOOST_PP_ENUM_PARAMS(ASIOSIG_n,A))> signal_base;
public:
  typedef ASIOSIG_typename signal_base::signature_type signature_type;
  typedef ASIOSIG_typename signal_base::result_type    result_type;
  
  template<unsigned N>
  class arg :public signal_base::template arg<N> {};

  static unsigned const arity=ASIOSIG_n;

  typedef ASIOSIG_typename signal_base::slot_function_type slot_function_type;
  typedef ASIOSIG_typename signal_base::slot_type          slot_type;
  typedef ASIOSIG_typename signal_base::extended_slot_function_type extended_slot_function_type;
  typedef ASIOSIG_typename signal_base::extended_slot_type          extended_slot_type;
  
  typedef boost::signals2::connection     connection;
  typedef boost::asio::io_service::strand strand;

  // structors 
  explicit asio_signal(boost::asio::io_service &io):m_service(io) {}
  ~asio_signal() {}

  // asio utilities
  boost::asio::io_service &service() {
    return m_service;
  }
  strand new_strand() {
    return strand(service());
  }

  // signal operator
  void operator()(ASIOSIG_fn_parms) {
    m_signal(ASIOSIG_fn_args);
  }

  // connection observers 
  bool empty() const {
    return m_signal.empty();
  }
  size_t num_slots() const {
    return m_signal.num_slots();
  }

  // connection utilities
  connection connect(slot_type const &cb) {
    return async_connect_impl(service(), cb);
  }
  connection connect_extended(extended_slot_type const &cb) {
    return async_ext_connect_impl(service(), cb);
  }
  connection strand_connect(strand &s, slot_type const &cb) {
    return async_connect_impl(s, cb);
  }
  connection strand_connect(slot_type const &cb) {
    strand s=new_strand();
    return strand_connect(s, cb);
  }
  connection strand_connect_extended(strand &s, extended_slot_type const &cb) {
    return async_ext_connect_impl(s, cb);
  }
  connection strand_connect_extended(extended_slot_type const &cb) {
    strand s=new_strand();
    return strand_connect_extended(s, cb);
  }
  
private:
  template<class Service>
  connection async_connect_impl(Service &s, slot_type const &cb) {
    return m_signal.connect(s.wrap(cb));
  }
  template<class Service>
  connection async_ext_connect_impl(Service &s, extended_slot_type const &cb) {
    return m_signal.connect_extended(s.wrap(cb));
  }

  boost::asio::io_service &m_service;
  signal_base              m_signal;
}; // class asio_signal<void (...)>

#  undef ASIOSIG_fn_comma
#  undef ASIOSIG_typename
#  undef ASIOSIG_fn_args
#  undef ASIOSIG_fn_parms
#  undef ASIOSIG_fn_parm
#  undef ASIOSIG_n

# endif // BOOST_PP_IS_ITERATING
#endif //  H_trex_utils_asio_signal
